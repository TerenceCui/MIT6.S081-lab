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

#define NBUCKET 13
#define LOCK_NAME_LEN 20

struct bucket {
  struct spinlock lock;
  struct buf head;
};

struct {
  struct spinlock lock;
  struct buf buf[NBUF];
  struct bucket hashtable[NBUCKET];
} bcache;



int hash(uint dev, uint blockno) {
  return (dev + blockno) % NBUCKET;
}



char bucket_lock_name[NBUCKET][LOCK_NAME_LEN];

void
binit(void)
{
  struct buf *b;
  struct bucket *bucket;

  initlock(&bcache.lock, "bcache_evict");
  for (int i = 0; i < NBUCKET; i++) {
    char *name = bucket_lock_name[i];
    snprintf(name, LOCK_NAME_LEN - 1, "bcache_bucket_%d", i);
    initlock(&bcache.hashtable[i].lock, name);
  }
  b = bcache.buf;
  for (int i = 0; i < NBUF; i++) {
    initsleeplock(&bcache.buf[i].lock, "buffer");
    bucket = bcache.hashtable + (i % NBUCKET);
    b->next = bucket->head.next;
    bucket->head.next = b;
    b++;
  }
}


#define LOCK_NAME_LEN 20
char bucket_lock_name[NBUCKET][LOCK_NAME_LEN];


// 查找设备dev上的块blockno是否在缓冲区缓存中
// 如果未找到，则分配一个缓冲区
// 无论哪种情况，都返回一个已锁定的缓冲区
static struct buf*
bget(uint dev, uint blockno)
{
  // 定义一个buf结构体指针b，用于遍历缓冲区链表
  struct buf *b, *prev_replace_buf;
  struct bucket *bucket, *prev_bucket, *cur_bucket;

  
  int index = hash(dev, blockno);
  // 获取哈希表中对应的桶
  bucket = bcache.hashtable + index;

  // 检查块是否已经在缓存中
  acquire(&bucket->lock);
  for (b = bucket->head.next; b; b = b->next) {
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bucket->lock);
      acquiresleep(&b->lock);
      return b;
    }
  }
  release(&bucket->lock);

  // 获取evict-lock
  acquire(&bcache.lock);

  // 再次检查当前桶
  // 如果在释放bucket-lock和获取evict-lock之间发生了另一个驱逐操作
  // 那么有可能块已经缓存在当前桶中，但进程不知道
  // 如果不进行检查，具有相同blockno的块可能存在于桶中
  acquire(&bucket->lock);
  for (b = bucket->head.next; b; b = b->next) {
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bucket->lock);
      release(&bcache.lock);
      acquiresleep(&b->lock);
      return b;
    }
  }
  release(&bucket->lock);

  // 块未缓存
  // 回收最近最少使用（LRU）的未使用缓冲区
  // 遍历每个桶和每个缓冲区，选择最近最少使用的缓冲区
  prev_replace_buf = (void*)0; // 要驱逐的缓冲区的前一个节点
  prev_bucket = (void*)0; // 之前锁定的桶，初始为空
  for (cur_bucket = bcache.hashtable; cur_bucket < bcache.hashtable + NBUCKET; cur_bucket++) {
    acquire(&cur_bucket->lock);
    int found = 0;
    for (b = &cur_bucket->head; b->next; b = b->next) {
      if (b->next->refcnt == 0 && 
      (!prev_replace_buf || b->next->timestamp < prev_replace_buf->next->timestamp)) {
        found = 1;
        prev_replace_buf = b;//last buffer before the one to be replaced
      }
    }
    if (!found) {
      release(&cur_bucket->lock);
    } else { 
      // 如果在桶中找到了较小的时间戳，释放之前持有的桶
      if (prev_bucket) {
        release(&prev_bucket->lock);
      }
      prev_bucket = cur_bucket;
    }
  }
  
  if (!prev_replace_buf) {
    panic("bget: no buffers");
  }
  b = prev_replace_buf->next;

  if (prev_bucket != bucket) {
    // 从其桶中删除要驱逐的缓冲区
    prev_replace_buf->next = b->next;
    release(&prev_bucket->lock);
    // 将缓冲区添加到桶中
    acquire(&bucket->lock);
    b->next = bucket->head.next;
    bucket->head.next = b;
  }

  // 更新缓冲区信息
  b->dev = dev;
  b->blockno = blockno;
  b->refcnt = 1;
  b->valid = 0;

  release(&bucket->lock);
  release(&bcache.lock);
  acquiresleep(&b->lock);
  return b;
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


// Release a locked buffer and update the timestamp
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  int index = hash(b->dev, b->blockno);
  acquire(&bcache.hashtable[index].lock);
  b->refcnt--;
  if (b->refcnt == 0) {
    b->timestamp = ticks;
  }
  release(&bcache.hashtable[index].lock);
}

void
bpin(struct buf *b) {
  int index = hash(b->dev, b->blockno);
  acquire(&bcache.hashtable[index].lock);
  b->refcnt++;
  release(&bcache.hashtable[index].lock);
}

void
bunpin(struct buf *b) {
  int index = hash(b->dev, b->blockno);
  acquire(&bcache.hashtable[index].lock);
  b->refcnt--;
  release(&bcache.hashtable[index].lock);
}


