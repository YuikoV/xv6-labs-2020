// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define NBUCKET 13  // 使用素数个桶，降低哈希冲突
#define HASH(id) (id % NBUCKET)


struct hashbuf {
  struct buf head;       // 哑节点（头节点）
  struct spinlock lock;  // 每个桶一个锁
};
struct {
  struct buf buf[NBUF];
  struct hashbuf buckets[NBUCKET];  // 哈希桶数组
} bcache;

void
binit(void) {
  struct buf* b;
  char lockname[16];

  // 初始化所有哈希桶
  for(int i = 0; i < NBUCKET; ++i) {
    snprintf(lockname, sizeof(lockname), "bcache_%d", i);
    initlock(&bcache.buckets[i].lock, lockname);
    
    // 初始化循环双向链表（头节点指向自己）
    bcache.buckets[i].head.prev = &bcache.buckets[i].head;
    bcache.buckets[i].head.next = &bcache.buckets[i].head;
  }

  // 将所有缓冲区放入桶 0（使用头插法）
  for(b = bcache.buf; b < bcache.buf + NBUF; b++) {
    b->next = bcache.buckets[0].head.next;
    b->prev = &bcache.buckets[0].head;
    initsleeplock(&b->lock, "buffer");
    bcache.buckets[0].head.next->prev = b;
    bcache.buckets[0].head.next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno) {
  struct buf* b;
  int bid = HASH(blockno);
  
  acquire(&bcache.buckets[bid].lock);

  // 第一步：检查是否已缓存
  for(b = bcache.buckets[bid].head.next; 
      b != &bcache.buckets[bid].head; 
      b = b->next) {
    if(b->dev == dev && b->blockno == blockno) {
      b->refcnt++;
      acquire(&tickslock);
      b->timestamp = ticks;
      release(&tickslock);
      release(&bcache.buckets[bid].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // 第二步：未缓存，需要分配一个缓冲区
  b = 0;
  struct buf* tmp;

  // 遍历所有桶，查找最近最少使用的未引用缓冲区
  for(int i = bid, cycle = 0; cycle != NBUCKET; 
      i = (i + 1) % NBUCKET) {
    ++cycle;
    
    // 如果不是当前桶，需要获取锁
    if(i != bid) {
      if(!holding(&bcache.buckets[i].lock))
        acquire(&bcache.buckets[i].lock);
      else
        continue;  // 避免死锁：已经持有该锁就跳过
    }

    // 在当前桶中查找 LRU 缓冲区
    for(tmp = bcache.buckets[i].head.next; 
        tmp != &bcache.buckets[i].head; 
        tmp = tmp->next) {
      if(tmp->refcnt == 0 && 
         (b == 0 || tmp->timestamp < b->timestamp))
        b = tmp;
    }

    if(b) {
      // 找到了！如果是从其他桶"偷"来的，需要移动到目标桶
      if(i != bid) {
        // 从原桶中移除
        b->next->prev = b->prev;
        b->prev->next = b->next;
        release(&bcache.buckets[i].lock);

        // 插入到目标桶（头插法）
        b->next = bcache.buckets[bid].head.next;
        b->prev = &bcache.buckets[bid].head;
        bcache.buckets[bid].head.next->prev = b;
        bcache.buckets[bid].head.next = b;
      }

      // 初始化缓冲区
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      
      acquire(&tickslock);
      b->timestamp = ticks;
      release(&tickslock);
      
      release(&bcache.buckets[bid].lock);
      acquiresleep(&b->lock);
      return b;
    } else {
      // 当前桶没找到，继续下一个
      if(i != bid)
        release(&bcache.buckets[i].lock);
    }
  }

  panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf* b) {
  if(!holdingsleep(&b->lock))
    panic("brelse");

  int bid = HASH(b->blockno);
  releasesleep(&b->lock);

  acquire(&bcache.buckets[bid].lock);
  b->refcnt--;
  
  // 更新时间戳用于 LRU
  // 不再需要头插法移动节点位置
  acquire(&tickslock);
  b->timestamp = ticks;
  release(&tickslock);
  
  release(&bcache.buckets[bid].lock);
}

void 
bpin(struct buf* b) {
  int bid = HASH(b->blockno);
  acquire(&bcache.buckets[bid].lock);
  b->refcnt++;
  release(&bcache.buckets[bid].lock);
}

void 
bunpin(struct buf* b) {
  int bid = HASH(b->blockno);
  acquire(&bcache.buckets[bid].lock);
  b->refcnt--;
  release(&bcache.buckets[bid].lock);
}


