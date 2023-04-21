

class MinimalTaskModel():

    def __init__(self,TaskId,TaskPriority,status):
        super().__init__('minimal_task')
        self.TaskId=TaskId
        self.TaskPriority=TaskPriority
        self.status=status
        self.desired_status='Completed'
