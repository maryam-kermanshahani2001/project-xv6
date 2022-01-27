#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "x86.h"
#include "proc.h"
#include "spinlock.h"


// An enum default value is always zero
enum schedPolicy policy;


struct {
  struct spinlock lock;
  struct proc proc[NPROC];
} ptable;

static struct proc *initproc;
struct spinlock thread;

int nextpid = 1;
extern void forkret(void);
extern void trapret(void);

static void wakeup1(void *chan);

void
pinit(void)
{
  initlock(&ptable.lock, "ptable");
  initlock(&thread, "thread");
}

// Must be called with interrupts disabled
int
cpuid() {
  return mycpu()-cpus;
}

// Must be called with interrupts disabled to avoid the caller being
// rescheduled between reading lapicid and running through the loop.
struct cpu*
mycpu(void)
{
  int apicid, i;
  
  if(readeflags()&FL_IF)
    panic("mycpu called with interrupts enabled\n");
  
  apicid = lapicid();
  // APIC IDs are not guaranteed to be contiguous. Maybe we should have
  // a reverse map, or reserve a register to store &cpus[i].
  for (i = 0; i < ncpu; ++i) {
    if (cpus[i].apicid == apicid)
      return &cpus[i];
  }
  panic("unknown apicid\n");
}

// Disable interrupts so that we are not rescheduled
// while reading proc from the cpu structure
struct proc*
myproc(void) {
  struct cpu *c;
  struct proc *p;
  pushcli();
  c = mycpu();
  p = c->proc;
  popcli();
  return p;
}

//PAGEBREAK: 32
// Look in the process table for an UNUSED proc.
// If found, change state to EMBRYO and initialize
// state required to run in the kernel.
// Otherwise return 0.
static struct proc*
allocproc(void)
{
  struct proc *p;
  char *sp;

  acquire(&ptable.lock);

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == UNUSED)
      goto found;

  release(&ptable.lock);
  return 0;

found:
  p->state = EMBRYO;
  p->pid = nextpid++;
  p->stackTop = -1; // initialize stackTop to -1 (illegal value)
  p->threads = -1; // initalize threads to -1 (illegal value)
  
  release(&ptable.lock);

  // Allocate kernel stack.
  if((p->kstack = kalloc()) == 0){
    p->state = UNUSED;
    return 0;
  }
  sp = p->kstack + KSTACKSIZE;

  // Leave room for trap frame.
  sp -= sizeof *p->tf;
  p->tf = (struct trapframe*)sp;

  // Set up new context to start executing at forkret,
  // which returns to trapret.
  sp -= 4;
  *(uint*)sp = (uint)trapret;

  sp -= sizeof *p->context;
  p->context = (struct context*)sp;
  memset(p->context, 0, sizeof *p->context);
  p->context->eip = (uint)forkret;


  p->sleeping_t = 0;
  p->runnable_t = 0;
  p->running_t = 0;
  p->creation_t = ticks;

  p->priority = DEFAULT_PRIORITY;

  return p;}

//PAGEBREAK: 32
// Set up first user process.
void
userinit(void)
{
  struct proc *p;
  extern char _binary_initcode_start[], _binary_initcode_size[];

  p = allocproc();
  
  initproc = p;
  if((p->pgdir = setupkvm()) == 0)
    panic("userinit: out of memory?");
  inituvm(p->pgdir, _binary_initcode_start, (int)_binary_initcode_size);
  p->sz = PGSIZE;
  memset(p->tf, 0, sizeof(*p->tf));
  p->tf->cs = (SEG_UCODE << 3) | DPL_USER;
  p->tf->ds = (SEG_UDATA << 3) | DPL_USER;
  p->tf->es = p->tf->ds;
  p->tf->ss = p->tf->ds;
  p->tf->eflags = FL_IF;
  p->tf->esp = PGSIZE;
  p->tf->eip = 0;  // beginning of initcode.S

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  // this assignment to p->state lets other cores
  // run this process. the acquire forces the above
  // writes to be visible, and the lock is also needed
  // because the assignment might not be atomic.
  acquire(&ptable.lock);

  p->state = RUNNABLE;

  release(&ptable.lock);
}

// Grow current process's memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  // sz is a variable in proc struct that maintain the size of memory
  uint sz;
  struct proc *curproc = myproc();

  acquire(&thread);
  sz = curproc->sz;

  if(n > 0){
    // grow the memory by this function
    if((sz = allocuvm(curproc->pgdir, sz, sz + n)) == 0){
      release(&thread);
      return -1;
    }
  } else if(n < 0){
    if((sz = deallocuvm(curproc->pgdir, sz, sz + n)) == 0){
      release(&thread);
      return -1;
  }
  }
  curproc->sz = sz;
  acquire(&ptable.lock);
  // we should update sz in all threads due to existence of clone system call
  struct proc *p;
  int numberOfChildren;
  // check if it is a child or parent
  if (curproc->threads == -1) // child
  {
    // update parent sz
    curproc->parent->sz = curproc->sz;
    // -2 is because of updating parent along with one child
    numberOfChildren = curproc->parent->threads - 2;
    if (numberOfChildren <= 0)
    {
      release(&ptable.lock);
      release(&thread);
      switchuvm(curproc);
      return 0;
    }
     else
      for(p = ptable.proc; p < &ptable.proc[NPROC];p++){
        // if both of them has same father and they are thread
      if(p!=curproc && p->parent == curproc->parent && p->threads == -1){
        p->sz = curproc->sz;
        numberOfChildren--;
      }
    }
    }
  else{ // is not a child
    numberOfChildren = curproc->threads - 1;
    if(numberOfChildren <= 0){
      release(&ptable.lock);
      release(&thread);
      switchuvm(curproc);
      return 0;
    }
    else
      for(p = ptable.proc; p < &ptable.proc[NPROC];p++){
        if(p->parent == curproc && p->threads == -1){
          p->sz = curproc->sz;
          numberOfChildren--;

        }
      }
    
  }
  
  
  release(&ptable.lock);
  release(&thread);
  switchuvm(curproc);
  return 0;

}
// Create a new process copying p as the parent.
// Sets up stack to return as if from system call.
// Caller must set state of returned proc to RUNNABLE.
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *curproc = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy process state from proc.
  if((np->pgdir = copyuvm(curproc->pgdir, curproc->sz)) == 0){
    kfree(np->kstack);
    np->kstack = 0;
    np->state = UNUSED;
    return -1;
  }
  np->sz = curproc->sz;
  np->parent = curproc;
  *np->tf = *curproc->tf;
  np->stackTop = curproc->stackTop;
  np->threads = 1; // in fork system

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

  for(i = 0; i < NOFILE; i++)
    if(curproc->ofile[i])
      np->ofile[i] = filedup(curproc->ofile[i]);
  np->cwd = idup(curproc->cwd);

  safestrcpy(np->name, curproc->name, sizeof(curproc->name));

  pid = np->pid;

  acquire(&ptable.lock);

  np->state = RUNNABLE;

  release(&ptable.lock);

  return pid;
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait() to find out it exited.
void
exit(void)
{
  struct proc *curproc = myproc();
  struct proc *p;
  int fd;

  if(curproc == initproc)
    panic("init exiting");

  // Close all open files.
  for(fd = 0; fd < NOFILE; fd++){
    if(curproc->ofile[fd]){
      fileclose(curproc->ofile[fd]);
      curproc->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(curproc->cwd);
  end_op();
  curproc->cwd = 0;

  acquire(&ptable.lock);

  // if a child calls exit
  if(curproc->threads == -1){ 
    curproc->parent->threads--;
  }
  
  // Parent might be sleeping in wait().
  wakeup1(curproc->parent);

  // Pass abandoned children to init.
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->parent == curproc){
      p->parent = initproc;
      if(p->state == ZOMBIE)
        wakeup1(initproc);
    }
  }

  // Jump into the scheduler, never to return.
  curproc->state = ZOMBIE;
  curproc->termination_t=ticks;
  sched();
  panic("zombie exit");
}
// scan in ptable  
int
check_pgdir_share(struct proc *process)
{
  struct proc *p;
  for(p = ptable.proc; p < &ptable.proc[NPROC];p++){
    // the page directories are the same for 2 brother or sister or the child
    // return 0 means that I don't free the page directory cause there is a thread that has the same page directory
    if(p != process && p->pgdir == process->pgdir)
      return 0;

  }
  return 1;
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(void)
{
  struct proc *p;
  int havekids, pid;
  struct proc *curproc = myproc();
  
  acquire(&ptable.lock);
  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->parent != curproc)
        continue;
      // wait must wait for processes and thread_wait must wait for threads 
      if (p->threads == -1)
        continue;

      havekids = 1;
      if(p->state == ZOMBIE){
        // Found one.
        pid = p->pid;
        kfree(p->kstack);
        p->kstack = 0;
        // freevm(p->pgdir);
        if(check_pgdir_share(p))
          freevm(p->pgdir);
        
        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        p->state = UNUSED;

        p->stackTop = -1;
        p->pgdir = 0;
        p->threads = -1;

        release(&ptable.lock);
        return pid;
      }
    }
    // No point waiting if we don't have any children.
    if(!havekids || curproc->killed){
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit.  (See wakeup1 call in proc_exit.)
    sleep(curproc, &ptable.lock);  //DOC: wait-sleep
  }
}


void switch_process(struct cpu *c, struct proc *p)
{
  // Switch to chosen process.  It is the process's job
  // to release ptable.lock and then reacquire it
  // before jumping back to us.
  c->proc = p;
  switchuvm(p);
  p->state = RUNNING;

  swtch(&(c->scheduler), p->context);
  switchkvm();

  // Process is done running for now.
  // It should have changed its p->state before coming back.
  c->proc = 0;
}

//PAGEBREAK: 42
// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run
//  - swtch to start running that process
//  - eventually that process transfers control
//      via swtch back to the scheduler.
void
scheduler(void)
{
  struct proc *p;

  struct proc *highest_p = 0; // runnable process with highest priority
  int hasRunnable = 0;        // Whether there exists a runnable process or not


  struct cpu *c = mycpu();
  c->proc = 0;
  
  for(;;){
    // Enable interrupts on this processor.
    sti();
    acquire(&ptable.lock);

    struct proc * itP;
    switch (policy)
    {
      int queueNumber;

      case DEFAULT:
      case ROUND_ROBIN:
        for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
        {
          if (p->state != RUNNABLE)
            continue;
          switch_process(c, p);
        }
        break;

      case PRIORITY:
        hasRunnable = 0;
        for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
        {
          if (p->state == RUNNABLE)
          {
            highest_p = p;
            hasRunnable = 1;
            break;
          }
        }

        for (p = ptable.proc; p < &ptable.proc[NPROC]; p++) // finding the process with highest priority
        {
          if (p->state != RUNNABLE)
            continue;
          if (p->priority < highest_p->priority)
            highest_p = p;
        }

        if (hasRunnable)
        {
          switch_process(c, highest_p);
        }
        break;

      case MULTILAYRED_PRIORITY:
        hasRunnable = 0;
        queueNumber=7;
        for (itP = ptable.proc; itP < &ptable.proc[NPROC]; itP++){
          if(itP->state!=RUNNABLE){
            continue;
          }
          hasRunnable=1;
          if(itP->queue < queueNumber){
            queueNumber=itP->queue;
            highest_p= itP;
          }else if(itP->queue==highest_p->queue && highest_p->queueIndex>itP->queueIndex){
            highest_p= itP;//since it has a less index in the list
          }
        }
        if (hasRunnable)
        {
          switch_process(c, highest_p);
        }
        break;
      case MULTILAYRED_PRIORITY_WITH_RULES:
        hasRunnable = 0;
        queueNumber=7;
        for (itP = ptable.proc; itP < &ptable.proc[NPROC]; itP++){
          if(itP->state!=RUNNABLE){
            continue;
          }
          hasRunnable=1;
          if(itP->queue < queueNumber){
            queueNumber=itP->queue;
            highest_p= itP;
          }else if(itP->queue==highest_p->queue && highest_p->queueIndex>itP->queueIndex){
            highest_p= itP;//since it has a less index in the list
          }
        }
        if (hasRunnable)
        {
          switch_process(c, highest_p);
        }
        break;
    }
    
    release(&ptable.lock);

  }
}

// Enter scheduler.  Must hold only ptable.lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->ncli, but that would
// break in the few places where a lock is held but
// there's no process.
void
sched(void)
{
  int intena;
  struct proc *p = myproc();

  if(!holding(&ptable.lock))
    panic("sched ptable.lock");
  if(mycpu()->ncli != 1)
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(readeflags()&FL_IF)
    panic("sched interruptible");
  intena = mycpu()->intena;
  swtch(&p->context, mycpu()->scheduler);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  acquire(&ptable.lock);  //DOC: yieldlock
  myproc()->state = RUNNABLE;
  sched();
  release(&ptable.lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch here.  "Return" to user space.
void
forkret(void)
{
  static int first = 1;
  // Still holding ptable.lock from scheduler.
  release(&ptable.lock);

  if (first) {
    // Some initialization functions must be run in the context
    // of a regular process (e.g., they call sleep), and thus cannot
    // be run from main().
    first = 0;
    iinit(ROOTDEV);
    initlog(ROOTDEV);
  }

  // Return to "caller", actually trapret (see allocproc).
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void
sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  
  if(p == 0)
    panic("sleep");

  if(lk == 0)
    panic("sleep without lk");

  // Must acquire ptable.lock in order to
  // change p->state and then call sched.
  // Once we hold ptable.lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup runs with ptable.lock locked),
  // so it's okay to release lk.
  if(lk != &ptable.lock){  //DOC: sleeplock0
    acquire(&ptable.lock);  //DOC: sleeplock1
    release(lk);
  }
  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;

  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  if(lk != &ptable.lock){  //DOC: sleeplock2
    release(&ptable.lock);
    acquire(lk);
  }
  if(policy==MULTILAYRED_PRIORITY_WITH_RULES){
    setPriority(1);//change it to the highest priorityS
  }
}

//PAGEBREAK!
// Wake up all processes sleeping on chan.
// The ptable lock must be held.
static void
wakeup1(void *chan)
{
  struct proc *p;

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == SLEEPING && p->chan == chan)
      p->state = RUNNABLE;
}

// Wake up all processes sleeping on chan.
void
wakeup(void *chan)
{
  acquire(&ptable.lock);
  wakeup1(chan);
  release(&ptable.lock);
}

// Kill the process with the given pid.
// Process won't exit until it returns
// to user space (see trap in trap.c).
int
kill(int pid)
{
  struct proc *p;

  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->pid == pid){
      p->killed = 1;
      // Wake process from sleep if necessary.
      if(p->state == SLEEPING)
        p->state = RUNNABLE;
      release(&ptable.lock);
      return 0;
    }
  }
  release(&ptable.lock);
  return -1;
}

//PAGEBREAK: 36
// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused",
  [EMBRYO]    "embryo",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  int i;
  struct proc *p;
  char *state;
  uint pc[10];

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    cprintf("%d %s %s", p->pid, state, p->name);
    if(p->state == SLEEPING){
      getcallerpcs((uint*)p->context->ebp+2, pc);
      for(i=0; i<10 && pc[i] != 0; i++)
        cprintf(" %p", pc[i]);
    }
    cprintf("\n");
  }
}

int
getProcCount(void) 
{
  int procCount = 0;

    struct proc *p;
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if (p->state == UNUSED) 
        continue;
      else
      {
        procCount++;       
      }
      
  }
  return procCount;

}

int thread_create(void *stack) {
  int i, pid;
  // np is the new process
  struct proc *np;
  struct proc *curproc = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }
  // increase # of threads for parent, default value for threads for child is -1
  curproc->threads++;
  
  // remember stack grows downwards thus the stacktop will be in the address given by parent + one page size
  np->stackTop= (int)((char *)stack + PGSIZE);

  // might be at the middle of changing address space in another thread
    acquire(&ptable.lock);
    np->pgdir = curproc->pgdir;
    np->sz = curproc->sz;
    release(&ptable.lock);

    int bytesOnStack = curproc->stackTop - curproc->tf->esp;
    np->tf->esp = np->stackTop - bytesOnStack;
    // copy stack of parent to stack of child
    memmove((void*)np->tf->esp, (void*)curproc->tf->esp, bytesOnStack);
/* remove next comment lines because this is a thread not a process
  // // Copy process state from proc.
  // if((np->pgdir = copyuvm(curproc->pgdir, curproc->sz)) == 0){
  //   kfree(np->kstack);
  //   np->kstack = 0;
  //   np->state = UNUSED;
  //   return -1;
  // }
   np->sz = curproc->sz;
*/
  np->parent = curproc;

  // copying all trapframe register values from p into new process
  *np->tf = *curproc->tf;

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

   // esp points to the top of the stack (esp is the stack pointer)
  np->tf->esp = np->stackTop - bytesOnStack;
  // ebp is the base pointer
  np->tf->ebp = np->stackTop - (curproc->stackTop - curproc->tf->ebp);
   // esp points to the top of the stack (esp is the stack pointer)
  for(i = 0; i < NOFILE; i++)
    if(curproc->ofile[i])
      np->ofile[i] = filedup(curproc->ofile[i]);
  np->cwd = idup(curproc->cwd);

  safestrcpy(np->name, curproc->name, sizeof(curproc->name));

  pid = np->pid;

  acquire(&ptable.lock);

  np->state = RUNNABLE;

  release(&ptable.lock);

  return pid;
}
// extern int readCount;



int thread_wait() {

  struct proc *p;
  int havekids, pid;
  struct proc *curproc = myproc();
  
  acquire(&ptable.lock);
  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->parent != curproc)
        continue;
      if (p->threads != -1) // remember thread_wait only waits for child threads not child processes
      {
        continue;
      }
      
      havekids = 1;
      if(p->state == ZOMBIE){
        // Found one.
        pid = p->pid;
        kfree(p->kstack);
        p->kstack = 0;

      // because that father and child page directories are the same
      // that process or child threads free the page directory but there are
      // processes and threads that still don't finish their works
        if(check_pgdir_share(p))
          freevm(p->pgdir);

        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        p->state = UNUSED;
        p->pgdir = 0;
        p->threads = -1;
        release(&ptable.lock);
        return pid;
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || curproc->killed){
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit.  (See wakeup1 call in proc_exit.)
    sleep(curproc, &ptable.lock);  //DOC: wait-sleep
  }
}


int setPriority(int newPriority)
{
  struct proc *p = myproc();

  if(policy==MULTILAYRED_PRIORITY || policy==MULTILAYRED_PRIORITY_WITH_RULES){
    if (newPriority > 0 && newPriority < 7)
    {
      p->priority = newPriority;
      p->queue=newPriority;//since our code works as less priority (more important -yes its a bit counter intuitive-)
      //now to find the index in the queue 
      int index = 0;
      struct proc *iterativep;
      for (iterativep = ptable.proc; iterativep < &ptable.proc[NPROC]; iterativep++){
        if (iterativep->queue==newPriority){
          if(index<=iterativep->queueIndex){
            index=iterativep->queueIndex+1;
          }
        }
      }
      return 0;
    }
    else{
      p->priority = 5;
      p->queue=newPriority;
      //now to find the index in the queue 
      struct proc *iterativep;
      int index = 0;
      for (iterativep = ptable.proc; iterativep < &ptable.proc[NPROC]; iterativep++){
        if (iterativep->queue==newPriority){
          if(index<=iterativep->queueIndex){
            index=iterativep->queueIndex+1;
          }
        }
      }
      return 0;
    }
  }
  if (newPriority > 0 && newPriority < 7)
  {
    p->priority = newPriority;
    return 0;
  }
  else{
    p->priority = 5;
    return 0;
  }
}

int changePolicy(int newPolicy)
{
  if (newPolicy >= 0 && newPolicy < 5)
  {
    policy = newPolicy;
    return 0;
  }
  else
    return -1;
}

int getTurnAroundTime(int pid)
{
  // if ((&ptable.proc[pid])->state == ZOMBIE)
  return (&ptable.proc[pid])->sleeping_t + (&ptable.proc[pid])->runnable_t + (&ptable.proc[pid])->running_t;
  // else
  //   return -1;
}

int getWaitingTime(int pid)
{
  return (&ptable.proc[pid])->sleeping_t + (&ptable.proc[pid])->runnable_t;
}

int getCBT(int pid)
{
  return (&ptable.proc[pid])->running_t;
}

// it is almost the same as wait but it gets process differents time such as turnAroundTime and waitingTime and CBT
int myWait(int *procTimes)
{
  struct proc *p;
  int havekids, pid;
  struct proc *curproc = myproc();

  acquire(&ptable.lock);
  for (;;)
  {
    // Scan through table looking for exited children.
    havekids = 0;
    for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    {
      if (p->parent != curproc)
        continue;
      havekids = 1;
      if (p->state == ZOMBIE)
      {
        // Found one.
        // store process times for further calculations
        int turnAroundTime = getTurnAroundTime(p->pid);
        int waitingTime = getWaitingTime(p->pid);
        int cbt = getCBT(p->pid);

        procTimes[0] = turnAroundTime;
        procTimes[1] = waitingTime;
        procTimes[2] = cbt;
        procTimes[3] = p->priority;

        pid = p->pid;
        kfree(p->kstack);
        p->kstack = 0;
        freevm(p->pgdir);
        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        p->state = UNUSED;

        // // Reset time spent in each state, for the next call.
        // p->sleeping_t = 0;
        // p->runnable_t = 0;
        // p->running_t = 0;
        release(&ptable.lock);
        return pid;
      }
    }

    // No point waiting if we don't have any children.
    if (!havekids || curproc->killed)
    {
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit.  (See wakeup1 call in proc_exit.)
    sleep(curproc, &ptable.lock); //DOC: wait-sleep
  }

  
}

void updateStateDurations()
{
  struct proc *p;
  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
  {
    switch (p->state)
    {
    case SLEEPING:
      p->sleeping_t++;
      break;

    case RUNNABLE:
      p->runnable_t++;
      break;

    case RUNNING:
      p->running_t++;
      break;

    default:
      break;
    }
  }
}

int existsBetterProccess(){
//no need to check if the policy is 3 or 4 since its checked before calling the function
  int change = 0;
  struct proc* p;
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->state!=RUNNABLE){
      continue;
    }
    if(p->queue<myproc()->queue){
      change=1;
    }else if(p->queue==myproc()->queue && p->queueIndex<myproc()->queueIndex ){
      change=1;
    }
  }

  return change;

}