
template <class T> class TaskAssignmentSystem {
public:
  TaskAssignmentSystem(T *task_assigner) : task_assigner_(task_assigner) {}

private:
  T *task_assigner_;
};
