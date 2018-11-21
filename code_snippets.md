# Get Position and Orientation of the Robot
Whiöe doing some control

```python
t = 0
positions = []
orientations = []
times = []
while t < 10:
    # Limit the length of the measurement to 10
    if len(positions) > 10:
        positions = positions[:-1]
    # Get the base position in World frame
    positions.append(pybullet.getBasePositionAndOrientation(robot.id)[0])
    # Get the base orientation in World frame ( Euler Angles)
    orientations.append(
        pybullet.getEulerFromQuaternion(
            # Index 1 == Quaternions
            pybullet.getBasePositionAndOrientation(robot.id)[1]
        )
    )
    # Get the current time
    times.append(t)
    # Apply control law
    controlJoints(t, 0, 2, controller_Group1)
    controlJoints(t, 1, -2, controller_Group2)
    # For slowing down
    sleep(0.2)
    t += 0.2
```

# Add a control law

```python
# Define a control function for a given control Group (dict)
def controlJoints(time, shift, amplitude ,controlGroup):
    # We know the structure of the control group
    counter = 0
    for joint, controller in controlGroup.items():
        if counter == 0:
            reference = amplitude*0.3*np.pi*np.sin(time+shift)
        elif counter == 1:
            reference = amplitude*0.2*np.pi*np.sin(time+shift)
        else:
            reference = 0
        controller.setReference(reference)
``
