# MASONRY SIMULATOR

This is a small ad-hoc simulator based on the [Project Chrono](http://projectchrono.org) multi-physics C++ library. It performs simulation of structures made with blocks (masonry buildings etc.) using the non-smooth dynamics approach.

Contact A. Tasora or V. Beatini for details.


## How to compile it

Before using this project, you must clone and compile [Project Chrono](http://projectchrono.org). Then, compile this project. You will find a **myexe.exe**  executable in your bin/Release/ build directory.

## How to use it

This simulator is command-line operated. There is no graphical interface for input, sorry.

Command syntax:

    myexe  [brickfile.txt]

where 
- [brickfile.txt]  is your file containing the positions and shapes of the bricks (see the format below)

There are optional parameters that you can enter in the command line (one or more of the above):

    myexe  [brickfile.txt]  springs [springfile.txt]  motion_X [motionfilex.txt]   motion_Y [motionfiley.txt]    motion_Z [motionfilez.txt]  motion_dt  [dt]   motion_amplifier [a]  dt [dt]  T_max [t]  save_each [n]  snapshot_each [n] 

where 
- **[brickfile.txt]**  is your file containing the positions and shapes of the blocks (see the format below)
- **springs [springfile.txt]** defines an optional file containing spring bars between blocks (see the format below), ex.  *springs my_springs.txt* 
- **precomputed_contacts [contactfile.txt]** defines an optional file with precomputed contact positions and normals at t=0. If so, it overrides collision detection. Note: only for statics, do not use if structures must collapse or move, because contacts will stick in original places.
- **motion_X [motionfilex.txt]**  defines an optional file containing the displacement of the blocks marked as 'fixed', i.e. the ground blocks, to simulate earthquake (see the format below). Ex.: *motion_X my_motion.txt* 
- same for **motion_Y** and **motion_Z** , note Y is up.
- **motion_dt  [dt]**  tells the timestep between samples in the motion files motion_X motion_Y and motion_Z. Default is 0.01 seconds.
- **motion_amplifier [a]**  tells how much the displacements in the motion files are multiplied. Default is 1, so displacements are kept unaltered, as in the input file.
- **dt [dt]**  is the timestep for the time integration; the larger the faster the simulation, but less precise. Default: 0.001 seconds.
- **iterations [n]** tells the max number of iterations for the complementarity solver. Default: 500.
- **T_max [t]**  tells when the simulator ends the simulation end exits. Default: 30 seconds.
- **save_each [n]** tells that the simulator will output on the disk a triplet of result files per each *n* timesteps: ex. *save_each 1*  will save  bodies00001.txt  bodies00002.txt ... contacts00001.txt contacts00002.txt ... springs00001.txt springs00002.txt  etc. Files will go into the working directory. Use 0 to disable saving. Default: save each 10 steps. 
- **snapshot_each [n]** tells that the simulator will save a snapshot of the 3D visualization each *n* timesteps.Snaphsots will go into the /video_capture directory. Default = 0 (do not save).
- **density [density]** defines the density of the blocks (kg/m^3). Default 1800 kg/m^3.
- **friction [f]** defines the friction coefficients for the blocks. Default 0.4.
- **compliance [c]** defines the compliance (m/N), inverse of stiffness, of contact points. Default 2e-8f m/N.
- **damping [r]** defines the Rayleigh damping in contacts. Default 0.2.
- **rolling_friction [rf]** optional, defines the rolling friction parameter (m), default 0, deactivated. Note: nonzero values causes 2x slower simulation.
- **spinning_friction [sf]** optional, defines the spinning friction parameter (m), default 0, deactivated. Note: nonzero values causes 2x slower simulation. If spinning_friction >0, it is convenient to have also rolling_friction>0.
- **rolling_compliance [rc]** optional, use only if rolling_friction >0. Units: rad/Nm. Default 0. 
- **spinning_compliance [sc]** optional, use only if spinning_friction >0. Units: rad/Nm. Default 0. 
- **penetrationrecovery [p]** sets the max speed (m/s) of separation when parts are compenetrating because of integration errors or model errors. Default 0.001 m/s. Do not use, or use high values, if non-zero compliance.
- **warmstart** can be 0 (off) or 1 (on). In some cases might speed up the convergence of the solver. Default 0.
- **SOR_solver** can be 0 (off) or 1 (on) to use the symmetric SOR solver. Default 0, so the default solver is the SPG Barzilai-Borwein, slower than SOR  but more precise.



This is an example of command line on my system:

    D:\build\masonry\Release>myexe 00spandrel2.txt motion_dt 0.01 motion_X DisHNE.txt motion_amplifier 1 iterations 20
    
    
## Input file format for bricks
  
The file for the bricks is an ASCII file, with a row per each block, each row being a sequence of comma separated values like this:

    [ID],[fixed],[visible], Fx, Fy, Fz, Refx,Refy,Refz, x1,y1,z1, x2,y2,z2, y3,y3,z3, .... (up to n x,y,z triplets, each per a block vertex)
    
where
- *[ID]* must be an unique integer identifier, usually an increasing integer per each row
- *[fixed]* must be 0 or 1, if 0 the block is free, if 1 the block is fixed to ground (fixed blocks will be used also to impose earthquake motion, if any), usually it is one fixed flat ground, andlot of no-fixed blocks but you can have multiple fixed 'ground' bodies.
- *[visible]* can be 0 or 1. If 1, block is invisible, but still partecipate to simulation.
- *Fx, Fy, Fz*, is the applied force, if any, otherwise use 0,0,0. Force is expressed in Newtons, in absolute reference frame, and applied to center of block.
- *Refx,Refy,Refz*, is the center of the reference coordinate syetem of the block, in absolute coordinates, in meters. It can be where you want.
- *x1,y1,z1, ... xn,yn,zn*    here you must put an arbitrary number of n vertexes defining the convex hull shape of the block. Position of the vertexes are expressed in meters, respect to the absolute coordinate. The order of the vertexes is not important: Chrono will rearrange them to make an unique convex hull. Note: no concave shapes are possible. Note: avoid too thin shapes. 

Example of a file with just two-blocks, one fixed, the other falling (two CR-terminated lines):

    0,1,1, 0,0,0, 0.247655,3.048388,0.4,  0.458039,2.772294,0.0,   0.0,2.8,0.0,  0.0,3.3,0.0,  0.518308,3.3,0.0,  0.518308,3.268648,0.0,  0.458039,2.772294,0.8,  0.518308,3.268648,0.8,  0.0,2.8,0.8,  0.0,3.3,0.8,  0.518308,3.3,0.8
    1,0,1,0,0,0,0.247655,4.048388,0.4,0.458039,3.772294,0.0,0.0,3.8,0.0,0.0,4.3,0.0,0.518308,4.3,0.0,0.518308,4.268648,0.0,0.458039,3.772294,0.8,0.518308,4.268648,0.8,0.0,3.8,0.8,0.0,4.3,0.8,0.518308,4.3,0.8

Note: if you need a brick to be made of multiple convex shapes (ex for a "L" shaped concave brick that can be decomposed in two convex sub-shapes), 
just use an asterisk in the list of vertexes to separate the vertexes of the n-th sub-shape from the vertexes of the (n+1)-th shape, ex. 

    ... 0,1,2, 3,4,1,*, 3,3,3, 4,2,3, ...

Note: if you need to override the default friction on a per-brick basis, add a ",f,[value]" at the end of the brick line, without spaces, for example here we override the friction of the second brick with a value of 0.0001: 

    0,1,1, 0,0,0, 0.247655,3.048388,0.4,  0.458039,2.772294,0.0,   0.0,2.8,0.0,  0.0,3.3,0.0,  0.518308,3.3,0.0,  0.518308,3.268648,0.0,  0.458039,2.772294,0.8,  0.518308,3.268648,0.8,  0.0,2.8,0.8,  0.0,3.3,0.8,  0.518308,3.3,0.8
    1,0,1,0,0,0,0.247655,4.048388,0.4,0.458039,3.772294,0.0,0.0,3.8,0.0,0.0,4.3,0.0,0.518308,4.3,0.0,0.518308,4.268648,0.0,0.458039,3.772294,0.8,0.518308,4.268648,0.8,0.0,3.8,0.8,0.0,4.3,0.8,0.518308,4.3,0.8,f,0.0001
	
You can set the friction with the "f" symbor per each sub-shape in concave bricks separated by asterisks, just add the f,[value] before the separating asterisk, ex.
    ... 0,1,2, 3,4,1,f,0001,*, 3,3,3, 4,2,3, 1,2,1,f,.0002,  
	
If two blocks with different frictions are in contact, the minimum of the two values is used in the combined contact.

## Input file format for springs
  
The (optional) file for the springs is an ASCII file, with a row per each spring, each row being a sequence of comma separated values like this:

    ID, IDbodyA, Ax,Ay,Az, IDbodyB, Bx,By,Bz,  k, L0
    
where:
- *[ID]* must be an unique integer identifier, usually an increasing integer per each row 
- *IDbodyA* is the identifier of the first of the two connected bodies
- *Bx,By,Bz*  is the position of the anchoring point of the spring on the first body, in absolute coordinate frame, in meters.
- *IDbodyB* is the identifier of the second of the two connected bodies
- *Bx,By,Bz*  is the position of the anchoring point of the spring on the second body, in absolute coordinate frame, in meters.
- *k* is the stiffness of the spring in Newton/meter 
- *L0* is the undeformed length of the spring (if the spring is created with two ends whose distance is different than L0, the spring will be pre-stressed, then)

## Input file format for precomputed contacts
  
Sometimes the collision detection algorithm has difficulties in placing the proper amount of contacts, exp.for cases with coplanar faces, degenerate geometries, etc. If so, a possibility is using a precomputed set of contacts, that one can obtain by hand work or via a CAD. Of course this can be used if the structure is static, i.e. it does not move or collapse, because contacts will stick to their bodies during motion even if there should be some sliding. 
If this feature is needed, the (optional) file for the precomputed contacts at t=0 is an ASCII file, with a row per each contact, each row being a sequence of comma separated values like this:

    IDbodyA, IDbodyB, Px,Py,Pz, Nx,Ny,Nz
    
where:
- *IDbodyA* is the identifier of the first of the two contacting bodies
- *IDbodyB* is the identifier of the first of the two contacting bodies
- *Px,Py,Pz*  is the position of the contact point in absolute reference, at t=0, in meters.
- *Nx,Ny,Nz*  is the normal of the contact point in absolute reference, at t=0, in meters, assumed pointing out from body A.



## Output file format for blocks
  
If save_each option is used, the simulator will output bodies00001.txt, bodies00002.txt etc. Each file contains the following infos, each row being a sequence of comma separated values like this:

    ID, x,y,z, e0,e1,e2,e3, r

where:
- *[ID]* is the unique identifier of the n-th body
- *x,y,z* is the position of the reference frame of the body respect to absolute coordinate frame, in meters,
- *e0,e1,e2,e3,* is the quaternion expressing the rotation of the body respect to absolute coordinate frame
- r is the absolute angle of rotation of the body in degrees, in the X-Y plane (useful only for 2D simulations, do not use for 3D)


## Output file format for contacts
  
If save_each option is used, the simulator will output contacts00001.txt, contacts00002.txt etc. Each file contains the following infos, each row being a sequence of comma separated values like this:

    IDa, IDb, x,y,z, Fx,Fy,Fz, Ux,Uv,Uz, Vx,Vy,Vz, Wx,Wy,Wz

where:
- *IDa* is the unique identifier of the first body in contact
- *IDb* is the unique identifier of the second body in contact
- *x,y,z* is the position of the reference frame of the contact respect to absolute coordinate frame, in meters
- *Fx,Fy,Fz,* is the contact force in Newtons, expressed in the reference frame of the contact (i.e. the Fx is the normal component, Fy and Fz are the tangent frictional components)
- *Ux,Uv,Uz, Vx,Vy,Vz, Wx,Wy,Wz* represent the 9 values of the rotation matrix expressing the reference frame of the contact respect to absolute coordinate frame. Note, the X direction of the contact frame is also the normal to the contact tangent plane. 
- r is the absolute angle of rotation of the body in degrees, in the X-Y plane (useful only for 2D simulations, do not use for 3D)


## Output file format for springs
  
If save_each option is used, the simulator will output springs00001.txt, springs00002.txt etc. Each file contains the following infos, each row being a sequence of comma separated values like this:

    IDs, F, Ax,Ay,Az, Bx,By,Bz 

where:
- *IDs* is the unique identifier of the spring,
- *F* is the spring force in Newtons, aligned to spring direction,
- *Ax,Ay,Az, Bx,By,Bz * represent the two ends of the spring, at that instant of the simulation, in the absolute coordinate frame.
