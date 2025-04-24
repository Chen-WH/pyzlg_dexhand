Joint Control Options
==================

The DexHand interface provides flexible ways to control the robotic hand, with options to optimize for different use cases.

move_joints Function
------------------

The primary interface for controlling the DexHand is the ``move_joints()`` function, which allows you to control specific joints with precise parameters:

.. code-block:: python

    # Standard joint control
    hand.move_joints(
        th_rot=30,     # Thumb rotation
        th_mcp=45,     # Thumb MCP flexion
        ff_mcp=60,     # Index finger MCP
        control_mode=ControlMode.IMPEDANCE_GRASP
    )

Additional Parameters
------------------

The ``move_joints()`` function supports several additional parameters to customize control:

Control Mode Selection
^^^^^^^^^^^^^^^^^^^^^

Control the behavior of the motors using different control strategies:

.. code-block:: python

    # Using specific control mode
    hand.move_joints(
        th_mcp=30,
        ff_mcp=45,
        control_mode=ControlMode.MIT_TORQUE
    )

Speed Control
^^^^^^^^^^^

Control motor velocities (in RPM):

.. code-block:: python

    # Single speed value for all motors
    hand.move_joints(
        th_mcp=30,
        speeds=15000  # Default is 15000 RPM
    )

    # Individual speeds for each motor
    hand.move_joints(
        th_mcp=30,
        ff_mcp=45,
        speeds=[12000, 12000, 15000, 15000, 15000, 15000, 
                15000, 15000, 15000, 15000, 15000, 15000]  # Must have 12 values
    )

Current Limits
^^^^^^^^^^^^

Set motor current limits (in mA):

.. code-block:: python

    # Single current value for all motors
    hand.move_joints(
        th_mcp=30,
        currents=20  # Default is 20 mA
    )

    # Individual currents for each motor
    hand.move_joints(
        th_mcp=30,
        ff_mcp=45,
        currents=[30, 30, 20, 20, 20, 20, 
                  20, 20, 20, 20, 20, 20]  # Must have 12 values
    )

Broadcast Mode (Default)
---------------------

The ``move_joints()`` function uses an efficient broadcast mode by default that optimizes communication with the hand:

.. code-block:: python

    # Broadcast mode is used by default (more efficient)
    hand.move_joints(
        th_mcp=30, 
        ff_mcp=45
    )
    
    # Explicitly disable broadcast mode if needed
    hand.move_joints(
        th_mcp=30, 
        ff_mcp=45,
        use_broadcast=False  # Use per-board commands instead
    )

**Benefits of broadcast mode:**

* More efficient: Sends a single message instead of multiple per-board commands
* Lower latency: All motors receive commands simultaneously
* Recommended for applications requiring high update rates
* Better synchronization of multi-joint movements

**When to use per-board commands (disable broadcast):**

* When you need to control specific boards individually
* In rare cases where broadcast causes timing issues
* When debugging specific boards or motors
* When very fine-grained control timing is required

**Implementation note:**

With broadcast mode (default), the system uses a more efficient communication protocol internally that sends one command affecting all motors, rather than individual commands to each board.

Error Handling
------------

The ``clear_errors()`` function accepts a ``use_global`` parameter that also enables more efficient communication:

.. code-block:: python

    # Efficiently clear all errors with a single command
    hand.clear_errors(use_global=True)  # Recommended

    # Or clear specific error states
    hand.clear_errors(clear_all=False, use_global=False)

Using this optimized approach is recommended in most cases for better performance.