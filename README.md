# jaco2_contact_detection
Messages and surface model for contact detection with the Kinova Jaco2

## jaco2_contact_msgs
The messages are only relevant for offline evaluation.

## jaco2_surface_model
This package contains the surface configuration for the Jaco 2 which can be loaded with cslibs_mesh_map via OpenMesh.
A surface model consists of a yaml-file specifying mesh-obj-files frame names and parent frame names of the meshes as 3 lists.
E.g.:

	meshes:
		- link1.obj
		- link2.obj
		- ...
	frame_ids:
		-link1
		-link2
		- ...
	parent_ids:
		- link1
		- link1
		- ...
