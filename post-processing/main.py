from pprint import pprint

import ecal
import ecal.measurement.hdf5


def main():
    meas_path = "/home/naoya/WildPose_v1.1/data/test/2022-11-30_15-17-43.373_measurement"
    channel_name = "rt/image_raw"
    cam_context_path = "/home/naoya/WildPose_v1.1/data/test/2022-11-30_15-17-43.373_measurement/cam_context.bin"
    out_path_rgb = "/home/naoya/WildPose_v1.1/data/test/2022-11-30_15-17-43.373_measurement/rgb/"
    out_path_raw = "/home/naoya/WildPose_v1.1/data/test/2022-11-30_15-17-43.373_measurement/raw/"

    meas = ecal.measurement.hdf5.Meas(meas_path)
    pprint(meas)


if __name__ == "__main__":
    main()