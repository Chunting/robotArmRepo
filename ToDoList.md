### TO DO LIST
- URMove
  - [X] Plot Path from CurrentPos to Target 
  - [X] Interpolate based on distance
  - [X] tweak acceleration/deacceleration to prevent sudden stops/jerks
  - [X] select correct Q Sol
  - [ ] remap to correct ranges/wrap to nearest joint position
- URKinematics
  - [X] figure out why the inverse is mirrored
  - [X] initial rotations on quaternion?
  - [ ] Joints to extend ofNode, Y or N?
- _URTool_
  - [ ] Create URTool class to manage the Toolpoint
  - [ ] create safe zone to prevent toolpoint from colliding with body.
- URWorkingZone
  - [X] Define a working zone and optimize movements to remain within that zone (Sort of done)
- Refactoring
  - [X] Rename Toolpoint -> TCP & targetTCP
  - [X] Remove redundant variables (e.g, targetPointPos -> tartgetTCP.position)
  - [X] Synchronize world scalar for Mocap & Robot
- Examples
  - Follow Path
  - [X] Fix Orientation with ofMatrix4x4
  - [ ] Add in accel & speed controls
  - [ ] Add in path interpolation based on time
<<<<<<< Updated upstream
  - [ ] Remove unncessary panels from GUI
=======
>>>>>>> Stashed changes
  - Follow Surface
  - [X] Integrate Robut with Geometry
  - [ ] Fix tool to be TCP on surface
  - [ ] Add in accel & speed controls
  - [ ] Add in path interpolation based on time
<<<<<<< Updated upstream
  - [ ] Remove unncessary panels from GUI
  - Follow Mocap
  - [ ] Fix Orientation with ofMatrix4x4
  - [ ] Add in accel & speed controls
  - [ ] Remove unncessary panels from GUI
  - Basic
  - [ ] Remove unncessary panels from GUI
=======
>>>>>>> Stashed changes
  
