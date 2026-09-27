# RobotAutomationStudio Code Map

This file exists to keep future edits small. Open the narrowest module first.

## App Shell

- `main.py`: application entry point and self-check.
- `app/application.py`: QApplication bootstrap and theme loading.
- `ui/main_window.py`: main navigation, project wiring, page ownership.
- `ui/widgets/editable_title.py`: topbar title label with double-click rename signal.
- `ui/styles/app.qss`, `ui/styles/app_dark.qss`: global Light/Dark styling.

## Robot And Simulation

- `ui/pages/robot_page.py`: main Robot tab layout, move/jog/position library commands.
- `ui/dialogs/simulation_detail_dialog.py`: Simulation Detail window, teaching preview, tool/gripper, virtual scene objects.
- `ui/widgets/robot_canvas.py`: robot/PyBullet preview widget and canvas interaction.
- `simulation/robot_model.py`: kinematic MG400 model, workspace limits, simulation snapshot.
- `simulation/pybullet_engine.py`: PyBullet body creation, render camera, object/tool visuals.

## Vision

- `ui/pages/vision_page.py`: Vision tab layout and workflow orchestration.
- `ui/widgets/image_view.py`: live image display, ROI drawing, result marker overlay.
- `ui/pages/vision_manuals.py`: calibration manual HTML text.
- `core/calibration_solver.py`: camera-to-robot matrix solving and calibration error math.
- `drivers/camera/base.py`: camera frame/config dataclasses and detection variable helpers.
- `drivers/camera/realsense_d405.py`: RealSense D405 capture, blob detection, checkerboard detection.

## Sequence And Analysis

- `ui/pages/sequence_page.py`: Sequence + Analysis tab UI, step editing, preview placement.
- `ui/widgets/tcp_monitor.py`: Robot TCP TX/RX/ERR packet monitor table.
- `core/sequence_engine.py`: sequence execution semantics and timing.

## Device And Project State

- `core/device_manager.py`: active robot/camera/gripper/plc selection and façade API.
- `core/project_manager.py`: project folder and JSON persistence.
- `core/models/state.py`: shared state dataclasses.

## Driver Boundaries

- `drivers/robot/*`: robot interfaces, MG400 TCP, simulation robot.
- `drivers/gripper/*`: gripper interfaces and simulation gripper.
- `drivers/io/*`, `drivers/plc/*`: IO and PLC abstractions.

## Edit Heuristics

- UI text/layout only: edit `ui/pages/*`, `ui/dialogs/*`, or `ui/widgets/*`.
- Calibration algorithm only: edit `core/calibration_solver.py`.
- RealSense detection behavior only: edit `drivers/camera/realsense_d405.py`.
- PyBullet visuals only: edit `simulation/pybullet_engine.py`.
- Robot reach/path warnings only: edit `simulation/robot_model.py`.
