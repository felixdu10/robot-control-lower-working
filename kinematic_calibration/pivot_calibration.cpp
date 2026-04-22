#include "atracsys_functions.h"
#include "./templated_classes/Templated_Transform.h"
namespace External {
#include "../Pivot.h"
#include "../Transform.h"
} // namespace External

External::Transform to_Transform(Transform<double> T) {
  External::Transform new_T;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      new_T.R_AB.matrixArray[i * 3 + j] = T.R.matrix[i][j];
    }
  }
  new_T.p_AB.matrixArray[0] = T.p.x;
  new_T.p_AB.matrixArray[1] = T.p.y;
  new_T.p_AB.matrixArray[2] = T.p.z;
  return new_T;
}

int main() {
  AtracsysTracker<double> atracsys("geometry100000.ini", "geometry999.ini");
  Measurement<double> m;
  std::vector<External::Transform> pivot_ts;
  for (int i = 0; i < 25; i++) {
    atracsys.getMeasurement(FOM2, m);
    pivot_ts.push_back(to_Transform(m.F_OM2));
  }
  External::Pivot pivot(pivot_ts);
  pivot.p_t.print();
}
