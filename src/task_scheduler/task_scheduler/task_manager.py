from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional
import threading

class TaskPhase(Enum):
    MOVE_TO_TOOL = auto()       # 前往工具点
    MOVE_TO_STATION = auto()    # 前往工位
    GRAB_TOOL = auto()          # 抓取工具
    RELEASE_TOOL = auto()       # 释放工具
    SORT_TOOLS = auto()         # 整理工具
    STANDBY = auto()            # 待机
    PATROL = auto()             # 巡逻

@dataclass
class Task:
    phase: TaskPhase
    tool: Optional[str] = None      # 工具名称
    location: Optional[str] = None  # 位置(工位或工具点)
    description: str = ""           # 任务描述

class TaskManager:
    def __init__(self):
        self._tasks: List[Task] = []
        self._lock = threading.Lock()
        self._initialize()
        
    def _initialize(self) -> None:
        """Initialize the task list with a standby task"""
        with self._lock:
            self._tasks = [self._create_task(TaskPhase.STANDBY, description="系统待机")]
    
    def _create_task(self, phase: TaskPhase, tool: Optional[str] = None, 
                   location: Optional[str] = None, description: str = "") -> Task:
        """Helper method to create a new task"""
        if not description:
            description = self._generate_description(phase, tool, location)
        return Task(phase, tool, location, description)
    
    def _generate_description(self, phase: TaskPhase, tool: Optional[str],
                           location: Optional[str]) -> str:
        """Generate description text for a task"""
        descriptions = {
            TaskPhase.MOVE_TO_TOOL: f"前往{tool}工具点" if tool else "前往工具点",
            TaskPhase.MOVE_TO_STATION: f"前往工位{location}",
            TaskPhase.GRAB_TOOL: f"抓取工具{tool}",
            TaskPhase.RELEASE_TOOL: f"在工位{location}释放{tool}",
            TaskPhase.SORT_TOOLS: "整理工具架",
            TaskPhase.STANDBY: "系统待机",
            TaskPhase.PATROL: "区域巡逻"
        }
        return descriptions.get(phase, "未知任务")
    
    def get_all_tasks(self) -> List[Task]:
        """Get a copy of all tasks in the list (for display purposes)"""
        with self._lock:
            return self._tasks.copy()
    
    def clear_tasks(self) -> None:
        """Clear all tasks and return to standby state"""
        with self._lock:
            self._initialize()

    def add_sort_task(self) -> None:
        """在任务列表末尾添加整理工具的任务"""
        with self._lock:
            # 如果当前最后一个任务已经是待机状态，先移除它
            if self._tasks and self._tasks[-1].phase == TaskPhase.STANDBY:
                self._tasks.pop()
            
            # 添加整理工具的任务序列
            self._tasks.append(
                self._create_task(TaskPhase.MOVE_TO_TOOL, description="前往工具整理点"))
            self._tasks.append(
                self._create_task(TaskPhase.SORT_TOOLS, description="开始整理工具"))
            

    def add_transport_task(self, tool: str, station: str) -> None:
        """添加运输任务并智能插入到合适位置
        
        Args:
            tool: 要运输的工具名称
            station: 目标工位名称
        """
        with self._lock:
            # 1. 创建四个原子任务
            move_to_tool = self._create_task(
                TaskPhase.MOVE_TO_TOOL, 
                tool=tool,
                description=f"前往{tool}工具点"
            )
            grab_tool = self._create_task(
                TaskPhase.GRAB_TOOL,
                tool=tool,
                description=f"抓取{tool}"
            )
            move_to_station = self._create_task(
                TaskPhase.MOVE_TO_STATION,
                location=station,
                description=f"前往{station}"
            )
            release_tool = self._create_task(
                TaskPhase.RELEASE_TOOL,
                tool=tool,
                location=station,
                description=f"在{station}释放{tool}"
            )

            # 2. 智能插入逻辑
            # 情况1：空队列
        if len(self._tasks) == 0:
            self._tasks.extend([move_to_tool, grab_tool, move_to_station, release_tool])
        
        # 情况2：当前是待机或巡逻状态
        elif self._tasks[0].phase in (TaskPhase.STANDBY, TaskPhase.PATROL):
            self._tasks.clear()
            self._tasks.extend([move_to_tool, grab_tool, move_to_station, release_tool])
        
        # 情况3：当前正在前往工具点或抓取工具
        elif self._tasks[0].phase in (TaskPhase.MOVE_TO_TOOL, TaskPhase.GRAB_TOOL):
            # 找到最后一个抓取工具任务的位置
            last_grab_index = -1
            for i, task in enumerate(self._tasks):
                if task.phase == TaskPhase.GRAB_TOOL:
                    last_grab_index = i
            
            # 在最后一个抓取任务后插入新的抓取任务
            if last_grab_index >= 0:
                self._tasks.insert(last_grab_index + 1, move_to_tool)
                self._tasks.insert(last_grab_index + 2, grab_tool)
            
            # 找到最后一个释放工具任务的位置
            last_release_index = -1
            for i, task in enumerate(self._tasks):
                if task.phase == TaskPhase.RELEASE_TOOL:
                    last_release_index = i
            
            # 在最后一个释放任务后插入移动和释放新工具
            if last_release_index >= 0:
                self._tasks.insert(last_release_index + 1, move_to_station)
                self._tasks.insert(last_release_index + 2, release_tool)
            else:
                # 如果没有释放任务，直接追加
                self._tasks.extend([move_to_station, release_tool])
        
        # 其他情况：直接追加到队列末尾
        else:
            self._tasks.extend([move_to_tool, grab_tool, move_to_station, release_tool])



    def complete_current_task(self) -> None:
        """完成当前任务(列表第一个任务)，所有任务前移一位"""
        with self._lock:
            if len(self._tasks) > 0:
                self._tasks.pop(0)  # 移除已完成的任务
                
                # 如果任务列表为空，添加待机任务
                if len(self._tasks) == 0:
                    self._tasks.append(
                        self._create_task(TaskPhase.STANDBY, description="系统待机"))


    def get_current_task(self) -> Optional[Task]:
        """获取当前任务(索引0的任务)"""
        with self._lock:
            return self._tasks[0] if self._tasks else None
        
    def print_tasks(self) -> str:
        """返回格式化后的任务列表字符串"""
        with self._lock:
            if not self._tasks:
                return "任务队列为空"
            
            output = "当前任务队列:\n"
            for idx, task in enumerate(self._tasks):
                output += f"[{idx}] {task.description}\n"
            return output

            
# def main():
#     # 创建任务管理器
#     task_manager = TaskManager()

#     # 初始状态（只有待机任务）
#     print("=== 初始任务列表 ===")
#     task_manager.print_tasks()


#     task_manager.add_transport_task("焊枪", "焊接工位")
#     print("\n添加焊枪运输任务后:")
#     task_manager.print_tasks()

#     task_manager.add_transport_task("剪刀", "剪刀工位")
#     print("\n添加剪刀运输任务后:")
#     task_manager.print_tasks()

#     # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()

#         # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()


#     # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()

#     # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()

#     # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()

#     # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()

#     # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()

#     # # 完成当前任务
#     print("\n=== 完成当前任务后 ===")
#     task_manager.complete_current_task()
#     task_manager.print_tasks()



# if __name__ == "__main__":
#     main()