import subprocess
import time
import os
import threading

arducopter_path = os.path.expanduser("~/ardupilot/ArduCopter")
base_port = 14550
port_step = 10
num_drones = 9
exit_flag = False
window_titles = []

def launch_drones():
    script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))

    for i in range(num_drones):
        sysid = i + 1
        instance = i
        port = base_port + (i * port_step)
        param_path = os.path.join(script_dir, f"drone_{sysid}.param")

        window_title = f"Drone_{sysid}"
        window_titles.append(window_title)

        command = (
            f"cd {arducopter_path} && "
            f"sim_vehicle.py -v ArduCopter "
            f"-f gazebo-iris --model JSON "
            f"-I{instance} "
            f"--wipe-eeprom "
            f"--add-param-file={param_path} "
            f"--out=udp:127.0.0.1:{port}; exec bash"
        )

        subprocess.Popen([
            "gnome-terminal",
            "--title", window_title,
            "--", "bash", "-c", command
        ])
        print(f"[INFO] Launched Drone {sysid} as '{window_title}' on port {port}")
        time.sleep(3)


def listen_for_input():
    global exit_flag
    while True:
        user_input = input(">>> ").strip().lower()
        if user_input == "exit":
            exit_flag = True
            break

def kill_all_terminals():
    print("[INFO] Closing all drone terminals...")
    for title in window_titles:
        # Find window ID by title
        try:
            window_id = subprocess.check_output(
                ["xdotool", "search", "--name", title]
            ).decode().strip().split("\n")[-1]

            # Send close signal (like Alt+F4)
            subprocess.run(["xdotool", "windowclose", window_id])
            print(f"[INFO] Closed window '{title}'")
        except subprocess.CalledProcessError:
            print(f"[WARN] Could not find window '{title}'")

    print("[INFO] All drone windows processed.")

if __name__ == "__main__":
    try:
        launch_drones()
        print("\n[INFO] Type 'exit' to close all drone terminals.")
        input_thread = threading.Thread(target=listen_for_input, daemon=True)
        input_thread.start()

        while not exit_flag:
            time.sleep(1)

        kill_all_terminals()

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received. Cleaning up...")
        kill_all_terminals()







#uncomment this and comment above code if you want just to launch the terminals and want to close them manually

# import subprocess
# import time
# import os

# arducopter_path = os.path.expanduser("~/ardupilot/ArduCopter")
# base_port = 14550
# port_step = 10
# num_drones = 9
# terminal_cmd = "gnome-terminal"

# for i in range(num_drones):
#     sysid = i + 1
#     instance = i
#     port = base_port + (i * port_step)
#     param_file = f"drone_{sysid}.param"
#     param_path = os.path.abspath(param_file)

#     # Write individual param file
#     with open(param_path, "w") as f:
#         f.write(f"MAV_SYSID {sysid}\n")

#     command = (
#         f'cd {arducopter_path} && '
#         f'echo "Launching Drone SYSID {sysid} on port {port}..." && '
#         f'sim_vehicle.py -v ArduCopter '
#         f'-f gazebo-iris --model JSON '
#         f'-I{instance} '
#         f'--wipe-eeprom '  # Reset EEPROM before applying params
#         f'--add-param-file={param_path} '
#         f'--out=udp:127.0.0.1:{port}; exec bash'
#     )

#     subprocess.Popen([
#         terminal_cmd,
#         "--title", f"Drone_{sysid}",
#         "--", "bash", "-c", command
#     ])

#     print(f"[INFO] Launched Drone {sysid} on port {port}")
#     time.sleep(5)


