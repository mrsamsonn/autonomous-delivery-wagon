import subprocess
import threading
import time
import os
import sys
from rich.console import Console
from rich.table import Table
from rich.live import Live

console = Console()
live = None  # Global reference to the Live display

def run_command(cmd):
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def build_workspace():
    console.print("[bold yellow]Building catkin workspace...[/bold yellow]")
    os.system("cd ~/catkin_ws && catkin_make")

def launch_stack():
    console.rule("[bold green]Launching ROS Stack[/bold green]")
    console.print("Starting roscore...")
    subprocess.Popen("roscore", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5)

    console.print("Starting usb_cam node...")
    build_workspace()
    run_command("rosrun usb_cam usb_cam_node")
    time.sleep(1)

    console.print("Launching lidar node...")
    run_command("rosrun unitree_lidar_ros unitree_lidar_ros_node")

def get_ros_nodes():
    try:
        result = subprocess.check_output("rosnode list", shell=True, stderr=subprocess.DEVNULL).decode().strip().split("\n")
        return [node for node in result if node and not node.startswith("/foxglove_")]
    except:
        return []

def check_node_status(node):
    try:
        output = subprocess.check_output(f"rosnode info {node}", shell=True, stderr=subprocess.DEVNULL).decode()
        return "Running" if "cannot communicate" not in output else "Dead"
    except:
        return "Dead"

def check_master_status():
    try:
        subprocess.check_output("rosnode list", shell=True, stderr=subprocess.DEVNULL)
        return True
    except:
        return False

def draw_node_table():
    table = Table(title="Live ROS Node Monitor", show_lines=True)
    table.add_column("Node", style="cyan")
    table.add_column("Status", style="magenta")

    nodes = get_ros_nodes()
    for node in nodes:
        status = check_node_status(node)
        table.add_row(node, "[green]Running[/green]" if status == "Running" else "[red]Dead[/red]")
    return table

def shut_down_nodes():
    global live
    if live:
        live.stop()  # Stop the live display before printing

    console.print("\n[bold red]Shutting down ROS nodes...[/bold red]")
    nodes = get_ros_nodes()
    for node in nodes:
        status = check_node_status(node)
        if status == "Dead":
            console.print(f"  [yellow]Node {node} is already stopped.[/yellow]")
        else:
            subprocess.call(f"rosnode kill {node}", shell=True)
            console.print(f"  [green]Shutting down node: {node}[/green]")
        time.sleep(0.2)

    console.print("\n[bold red]Stopping roscore...[/bold red]")
    os.system("pkill -f roscore")
    time.sleep(1)

    if not check_master_status():
        console.print("[green]roscore is successfully stopped.[/green]")
    else:
        console.print("[red]roscore is still running![/red]")

    console.print("\n[bold green]Shutdown complete. Exiting monitor.[/bold green]")
    sys.exit(0)

def wait_for_quit():
    while True:
        try:
            key = input().strip().lower()
            if key == "q":
                shut_down_nodes()
        except KeyboardInterrupt:
            shut_down_nodes()

def main():
    global live
    launch_stack()
    console.rule("[bold blue]Live ROS Node Monitor[/bold blue]")
    console.print("\n[bold]Press `q` then Enter to shut down all ROS nodes and quit.[/bold]\n", justify="center")

    threading.Thread(target=wait_for_quit, daemon=True).start()

    live = Live(draw_node_table(), refresh_per_second=0.5, screen=False)
    with live:
        while True:
            if not check_master_status():
                live.stop()  # Stop the live monitor once the ROS master is down
                console.print("\n[red]ROS master is down. Stopping live monitor.[/red]")
                break
            live.update(draw_node_table())
            time.sleep(1)

if __name__ == "__main__":
    main()

