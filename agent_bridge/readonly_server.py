"""
GRaCEmo ViRa Read-Only MCP Diagnostic Server.

Exposes six read-only tools.
No task creation, claiming, approval, status updates,
execution or result publishing.
"""

import json
import sys
from pathlib import Path
from typing import Optional

VIRA_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(VIRA_ROOT))

from mcp.server.mcpserver import MCPServer

from agent_bridge.storage import BridgeStorage
from agent_bridge.models import TaskStatus, TaskCategory, TaskPriority
from agent_bridge.config import (
    validate_task_id,
    BridgeValidationError,
    BridgePathSafetyError,
)

server = MCPServer("vira-readonly-diagnostics")
storage = BridgeStorage()


def response(data):
    return json.dumps(data, indent=2, default=str)


@server.tool(
    name="get_project_status",
    description="Read VIRA repository status and task statistics."
)
def get_project_status() -> str:
    try:
        return response(storage.get_project_status())
    except Exception as exc:
        return response({"status": "failed", "error": str(exc)})


@server.tool(
    name="list_pending_tasks",
    description="Read pending tasks with optional category and priority filters."
)
def list_pending_tasks(
    category: Optional[str] = None,
    priority: Optional[str] = None,
) -> str:
    try:
        if category is not None:
            category = TaskCategory(category.lower()).value
        if priority is not None:
            priority = TaskPriority(priority.lower()).value

        tasks = storage.list_tasks(
            status=TaskStatus.PENDING.value,
            category=category,
            priority=priority,
            limit=100,
        )

        return response({
            "count": len(tasks),
            "tasks": [task.model_dump() for task in tasks],
        })
    except ValueError as exc:
        return response({"status": "invalid_filter", "error": str(exc)})
    except Exception as exc:
        return response({"status": "failed", "error": str(exc)})


@server.tool(
    name="list_tasks",
    description="Read tasks with optional filters and bounded result count."
)
def list_tasks(
    status: Optional[str] = None,
    category: Optional[str] = None,
    priority: Optional[str] = None,
    limit: int = 50,
) -> str:
    try:
        status_val = TaskStatus(status.lower()).value if status else None
        category_val = TaskCategory(category.lower()).value if category else None
        priority_val = TaskPriority(priority.lower()).value if priority else None

        bounded_limit = min(max(int(limit), 1), 100)

        tasks = storage.list_tasks(
            status=status_val,
            category=category_val,
            priority=priority_val,
            limit=bounded_limit,
        )

        return response({
            "count": len(tasks),
            "tasks": [task.model_dump() for task in tasks],
        })
    except (ValueError, TypeError) as exc:
        return response({"status": "invalid_request", "error": str(exc)})
    except Exception as exc:
        return response({"status": "failed", "error": str(exc)})


@server.tool(
    name="read_task",
    description="Read an individual task by validated task ID."
)
def read_task(task_id: str) -> str:
    try:
        clean_id = validate_task_id(task_id)
        task = storage.get_task(clean_id)

        if not task:
            return response({"status": "not_found", "task_id": clean_id})

        return task.model_dump_json(indent=2)

    except (BridgeValidationError, BridgePathSafetyError) as exc:
        return response({"status": "invalid_request", "error": str(exc)})
    except Exception as exc:
        return response({"status": "failed", "error": str(exc)})


@server.tool(
    name="read_latest_result",
    description="Read the latest execution result, optionally filtered by task ID."
)
def read_latest_result(task_id: Optional[str] = None) -> str:
    try:
        clean_id = validate_task_id(task_id) if task_id else None
        result = storage.get_latest_result(clean_id)

        if not result:
            return response({"status": "not_found"})

        return result.model_dump_json(indent=2)

    except (BridgeValidationError, BridgePathSafetyError) as exc:
        return response({"status": "invalid_request", "error": str(exc)})
    except Exception as exc:
        return response({"status": "failed", "error": str(exc)})


@server.tool(
    name="check_approval_status",
    description="Read the approval status of an existing task."
)
def check_approval_status(task_id: str) -> str:
    try:
        clean_id = validate_task_id(task_id)
        approval = storage.check_approval(clean_id)

        if approval:
            return approval.model_dump_json(indent=2)

        task = storage.get_task(clean_id)

        if not task:
            return response({
                "status": "not_found",
                "task_id": clean_id,
            })

        return response({
            "task_id": clean_id,
            "status": "not_required",
        })

    except (BridgeValidationError, BridgePathSafetyError) as exc:
        return response({"status": "invalid_request", "error": str(exc)})
    except Exception as exc:
        return response({"status": "failed", "error": str(exc)})


def main():
    server.run(transport="stdio")


if __name__ == "__main__":
    main()
