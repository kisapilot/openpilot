from opendbc.car.common.numpy_fast import clip, interp
from cereal import car
from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from opendbc.car.hyundai.values import Buttons, CAR, DBC
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
      
# ajouatom
def enable_radar_tracks(CP, logcan, sendcan):
  # START: Try to enable radar tracks
  print("Try to enable radar tracks")
  if Params().get_bool("ExperimentalLongitudinalEnabled") and DBC[CP.carFingerprint]['radar'] == 'hyundai_kia_mando_front_radar_generated':
    rdr_fw_address = 0x7d0
    for i in range(10):
      print("O yes")
    try:
      for i in range(40):
        try:
          query = IsoTpParallelQuery(sendcan, logcan, CP.sccBus, [rdr_fw_address], [b'\x10\x07'], [b'\x50\x07'], debug=True)
          for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
            print("ecu write data by id ...")
            new_config = b"\x00\x00\x00\x01\x00\x01"
            dataId = b'\x01\x42'
            WRITE_DAT_REQUEST = b'\x2e'
            WRITE_DAT_RESPONSE = b'\x68'
            query = IsoTpParallelQuery(sendcan, logcan, CP.sccBus, [rdr_fw_address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug=True)
            query.get_data(0)
            print(f"Try {i+1}")
            break
          break
        except Exception as e:
          print(f"Failed {i}: {e}") 
    except Exception as e:
      print("Failed to enable tracks" + str(e))
  print("END Try to enable radar tracks")
  # END try to enable radar tracks