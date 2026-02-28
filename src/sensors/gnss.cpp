#include "gnss.h"
#include "3rdparty/utm.h"

bool GNSS::convert_utm() {
  char hemi{};
  double easting{}, northing{};
  long res = Convert_Geodetic_To_UTM(geodetic_[0] * M_PI / 180.0,
                                     geodetic_[1] * M_PI / 180.0, &utm_.zone,
                                     &hemi, &easting, &northing);
  utm_.north_hemi = hemi == 'N';
  utm_.xy = Eigen::Vector2d(easting, northing);

  return res == UTM_NO_ERROR;
}