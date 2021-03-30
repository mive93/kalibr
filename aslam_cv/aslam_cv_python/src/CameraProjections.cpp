#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/ExtendedUnifiedProjection.hpp>
#include <aslam/cameras/DoubleSphereProjection.hpp>
#include <aslam/cameras/NoDistortion.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <aslam/cameras/FovDistortion.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <boost/serialization/nvp.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
#include <sm/python/unique_register_ptr_to_python.hpp>

//#include <aslam/python/ExportDesignVariableAdapter.hpp>
//#include <aslam/backend/DesignVariableAdapter.hpp>
//#include <aslam/python/ExportAPrioriInformationError.hpp>

using namespace boost::python;
using namespace aslam::cameras;
//using namespace aslam::python;

// template<typename DERIVED_Y>
// void distort(const Eigen::MatrixBase<DERIVED_Y> & y) const;

#include <aslam/python/CameraProjections.tpp>
void exportFovDistortionFunctions() {
  class_<FovDistortion, boost::shared_ptr<FovDistortion> > distortion("FovDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<FovDistortion> >();

  exportGenericDistortionFunctions<FovDistortion>(distortion);
  distortion.def(init<double>(("FovDistortion(double w)")));
  distortion.def("w", &FovDistortion::w);
}

void exportRadialTangentialDistortionFunctions() {

  class_<RadialTangentialDistortion,
      boost::shared_ptr<RadialTangentialDistortion> > rtDistortion(
      "RadialTangentialDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<RadialTangentialDistortion> >();

  exportGenericDistortionFunctions<RadialTangentialDistortion>(rtDistortion);

  rtDistortion.def(
      init<double, double, double, double>(
          ("RadialTangentialDistortion(double k1, double k2, double p1, double p2)")));
  rtDistortion.def("k1", &RadialTangentialDistortion::k1);
  rtDistortion.def("k2", &RadialTangentialDistortion::k2);
  rtDistortion.def("p1", &RadialTangentialDistortion::p1);
  rtDistortion.def("p2", &RadialTangentialDistortion::p2);
  //rtDistortion.def("getLinesPack", &RadialTangentialDistortion::getLinesPack);
  //rtDistortion.def("distortionError", &RadialTangentialDistortion::distortionError);

  //exportGenericProjectionDesignVariable<RadialTangentialDistortion>("RadialTangentialDistortion");
}


void exportCameraProjections() {

  using namespace boost::python;
  using namespace aslam::cameras;
  //using namespace aslam::python;

  class_<NoDistortion, boost::shared_ptr<NoDistortion> > noDistortion(
      "NoDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<NoDistortion> >();

  exportGenericDistortionFunctions<NoDistortion>(noDistortion);

  class_<EquidistantDistortion, boost::shared_ptr<EquidistantDistortion> > equidistantDistortion(
      "EquidistantDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<EquidistantDistortion> >();

  equidistantDistortion.def(init<double, double, double, double>());
  exportGenericDistortionFunctions<EquidistantDistortion>(
      equidistantDistortion);
  //exportGenericProjectionDesignVariable<NoDistortion>("NoDistortion");

  exportRadialTangentialDistortionFunctions();
  exportFovDistortionFunctions();

  exportPinholeProjection<NoDistortion>("PinholeProjection");
///  exportPinholeProjection<RadialTangentialDistortion>(
///      "DistortedPinholeProjection");
///  exportPinholeProjection<EquidistantDistortion>(
///      "EquidistantPinholeProjection");
///  exportPinholeProjection<FovDistortion>(
///        "FovPinholeProjection");

///  exportOmniProjection<NoDistortion>("OmniProjection");
///  exportOmniProjection<RadialTangentialDistortion>("DistortedOmniProjection");
///  exportOmniProjection<FovDistortion>("FovOmniProjection");
///
///  exportExtendedUnifiedProjection<NoDistortion>("ExtendedUnifiedProjection");
///
///  exportDoubleSphereProjection<NoDistortion>("DoubleSphereProjection");


  // distortion:
  // exportAPrioriInformationError<aslam::backend::DesignVariableAdapter< RadialTangentialDistortion > >("RadialTangentialDistortionAPrioriInformationError");
  // exportAPrioriInformationError<aslam::backend::DesignVariableAdapter< PinholeProjection<RadialTangentialDistortion> > >("DistortedPinholeProjectionAPrioriInformationError");
  // exportAPrioriInformationError<aslam::backend::DesignVariableAdapter< OmniProjection<RadialTangentialDistortion> > >("DistortedOmniProjectionAPrioriInformationError");

}
