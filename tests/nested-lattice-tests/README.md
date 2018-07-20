Nested lattice tests Readme
===========================
1. Five types of simplified mcnp inputs(6 in total) are added to debug the issue that there is some geometry missing in UWNR model:

* **simplified_uwnr_lattice_w_trans**, **simplified_uwnr_lattice_wo_trans** - lattice map of the north core in UWNR model, w/ and w/o lattice transformation.
* **single_lat_w_trans**, **single_lat_wo_trans** - a single lattice in the geometry w/ and w/o lattice transformation.
* **single_pin_w_utrans**, **single_pin_wo_utrans** - a single pin model w/ and w/o universe transformation.
* **single_lat_w_rot_only**, **single_lat_w_rot_and_trans** - a single lattice in the geometry w/ rotation only and w/ both rotation and transformation.
* **single_pin_w_rot_only**, **single_pin_w_rot_and_trans** - single pin model w/ rotation only and w/ both rotation and transformation.

2. The test results of **mcnp2cad** are:
* There is geometry  missing in the lattice w/ transformation and it works  fine in the lattice w/o transformation.
* There is geometry  missing in the lattice w/ transformation and it works  fine in the lattice w/o transformation.
* Both work fine.
* TBD.
* TBD.
