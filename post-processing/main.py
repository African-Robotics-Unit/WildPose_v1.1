import os
import sys
from pprint import pprint

import ecal
import ecal.measurement.hdf5


def main():
    meas_path = "/data/test/2022-11-30_15-17-43.373_measurement"
    channel_name = "rt/image_raw"
    cam_context_path = os.path.join(meas_path, "cam_context.bin")
    out_path_rgb = os.path.join(meas_path, "rgb/")
    out_path_raw = os.path.join(meas_path, "raw/")

    meas = ecal.measurement.hdf5.Meas(meas_path)
    if not meas.is_ok():
        sys.exit(f'Error: cannot open {meas_path}')

    pprint(meas.get_channel_names())
    # convert image_raw into a video file


if __name__ == "__main__":
    main()