import dm_control.mujoco
import mujoco.viewer
import time

# Load the model from the XML file
m = dm_control.mujoco.MjModel.from_xml_path('example.xml')
d = dm_control.mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # initial camera settings
    viewer.cam.distance = 10
    viewer.cam.azimuth = 45  
    viewer.cam.elevation = -15  

    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # Control values for the wheels
        d.ctrl[0] = 5  # Front wheel
        d.ctrl[1] = 5  # Rear wheel

        # Step the simulation
        dm_control.mujoco.mj_step(m, d)

        # Sync the viewer
        viewer.sync()

        # Sleep for the remainder of the timestep
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
