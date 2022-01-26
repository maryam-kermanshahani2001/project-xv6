// Per-CPU state
struct cpu {
  uchar apicid;                // Local APIC ID
  struct context *scheduler;   // swtch() here to enter scheduler
  struct taskstate ts;         // Used by x86 to find stack for interrupt
  struct segdesc gdt[NSEGS];   // x86 global descriptor table
  volatile uint started;       // Has the CPU started?
  int ncli;                    // Depth of pushcli nesting.
  int intena;                  // Were interrupts enabled before pushcli?
  struct proc *proc;           // The process running on this cpu or null
};

extern struct cpu cpus[NCPU];
extern int ncpu;

//PAGEBREAK: 17
// Saved registers for kernel context switches.
// Don't need to save all the segment registers (%cs, etc),
// because they are constant across kernel contexts.
// Don't need to save %eax, %ecx, %edx, because the
// x86 convention is that the caller has saved them.
// Contexts are stored at the bottom of the stack they
// describe; the stack pointer is the address of the context.
// The layout of the context matches the layout of the stack in swtch.S
// at the "Switch stacks" comment. Switch doesn't save eip explicitly,
// but it is on the stack and allocproc() manipulates it.
struct context {
  uint edi;
  uint esi;
  uint ebx;
  uint ebp;
  uint eip;
};

enum procstate { UNUSED, EMBRYO, SLEEPING, RUNNABLE, RUNNING, ZOMBIE };

// Per-process state
struct proc {
  uint sz;                     // Size of process memory (bytes)
  pde_t* pgdir;                // Page table
  char *kstack;                // Bottom of kernel stack for this process
  enum procstate state;        // Process state
  int pid;                     // Process ID
  struct proc *parent;         // Parent process
  struct trapframe *tf;        // Trap frame for current syscall
  struct context *context;     // swtch() here to run process
  void *chan;                  // If non-zero, sleeping on chan
  int killed;                  // If non-zero, have been killed
  struct file *ofile[NOFILE];  // Open files
  struct inode *cwd;           // Current directory
  char name[16];        
  int stackTop;                // top of the stack
  int threads;                 // the number of threads using the same pgdir
  int queue;                  // Defines in which queue the process belongs


  int priority;       // Process priority, 1 being the highest, 6 lowest.

  int creation_t;    // Time which this process was created (turn into EMBRYO)
  int sleeping_t;    // Time spent being in SLEEPING state
  int runnable_t;    // Time spent being in RUNNABLE state ->this is specified as ready time in the project description, changed it to runnable time since it made more sence
  int running_t;     // Time spent being in RUNNING state
  int termination_t; // Time which this process was terminated (turn into ZOMBIE)

};
// Scheduling options
enum schedPolicy
{
  DEFAULT = 0,
  ROUND_ROBIN = 1,// this is the first part of the project
  PRIORITY = 2,//this is the second part of the project
  MULTILAYRED_PRIORITY = 3,// this is for the third part of the project
};
extern enum schedPolicy policy;
// Process memory is laid out contiguously, low addresses first:
//   text
//   original data and bss
//   fixed-size stack
//   expandable heap
