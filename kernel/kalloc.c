// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

struct ref_stru {
  struct spinlock lock;           // 保护引用计数的自旋锁
  int cnt[PHYSTOP / PGSIZE];     // 引用计数数组
} ref;
int
cowpage(pagetable_t pagetable, uint64 va)
{
  if(va >= MAXVA)
    return -1;
  
  pte_t* pte = walk(pagetable, va, 0);
  if(pte == 0)
    return -1;
  
  if((*pte & PTE_V) == 0)
    return -1;
  
  // 有PTE_F标志返回0（是COW页面），否则返回-1（不是）
  return (*pte & PTE_F ? 0 : -1);
}
void*
cowalloc(pagetable_t pagetable, uint64 va)
{
  if(va % PGSIZE != 0)
    return 0;

  uint64 pa = walkaddr(pagetable, va);
  if(pa == 0)
    return 0;

  pte_t* pte = walk(pagetable, va, 0);

  if(krefcnt((char*)pa) == 1) {
    // 情况1：只剩一个进程引用此物理地址
    // 直接修改PTE恢复写权限即可
    *pte |= PTE_W;
    *pte &= ~PTE_F;
    return (void*)pa;
  } else {
    // 情况2：多个进程引用此物理地址
    // 需要分配新页面并拷贝数据
    char* mem = kalloc();
    if(mem == 0)
      return 0;

    // 复制旧页面内容到新页
    memmove(mem, (char*)pa, PGSIZE);

    // 清除PTE_V，否则mappages会认为是重新映射
    *pte &= ~PTE_V;

    // 为新页面添加映射
    if(mappages(pagetable, va, PGSIZE, (uint64)mem, 
                (PTE_FLAGS(*pte) | PTE_W) & ~PTE_F) != 0) {
      kfree(mem);
      *pte |= PTE_V;
      return 0;
    }

    // 将原物理内存的引用计数减1
    kfree((char*)PGROUNDDOWN(pa));
    return mem;
  }
}
int
krefcnt(void* pa)
{
  return ref.cnt[(uint64)pa / PGSIZE];
}
int
kaddrefcnt(void* pa)
{
  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    return -1;
  
  acquire(&ref.lock);
  ++ref.cnt[(uint64)pa / PGSIZE];
  release(&ref.lock);
  return 0;
}
void
kinit()
{
  initlock(&kmem.lock, "kmem");
  initlock(&ref.lock, "ref");      // 初始化引用计数锁
  freerange(end, (void*)PHYSTOP);
}

void 
freerange(void *pa_start, void *pa_end) {
  char *p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE) {
    ref.cnt[(uint64)p / PGSIZE] = 1;  // 先设为1
    kfree(p);                         // kfree会减1变为0并释放
  }
}

// Free the page of physical memory pointed at by v,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  // 修改：只有当引用计数为0时才回收空间
  acquire(&ref.lock);
  if(--ref.cnt[(uint64)pa / PGSIZE] == 0) {
    release(&ref.lock);

    // 引用计数为0，真正释放内存
    r = (struct run*)pa;

    // Fill with junk to catch dangling refs.
    memset(pa, 1, PGSIZE);

    acquire(&kmem.lock);
    r->next = kmem.freelist;
    kmem.freelist = r;
    release(&kmem.lock);
  } else {
    release(&ref.lock);
    // 引用计数>0，只减计数不释放
  }
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void* kalloc(void) {
  struct run *r;
  acquire(&kmem.lock);
  r = kmem.freelist;
  if(r) {
    kmem.freelist = r->next;
    acquire(&ref.lock);
    ref.cnt[(uint64)r / PGSIZE] = 1;  // 新分配页面引用计数为1
    release(&ref.lock);
  }
  release(&kmem.lock);
  
  if(r)
    memset((char*)r, 5, PGSIZE);
  return (void*)r;
}
