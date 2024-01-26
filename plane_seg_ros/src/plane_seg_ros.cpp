#include <plane_seg_ros/Pass.h>
#include <pcl/io/ply_io.h>

// GridMapRosConverter includes cv_bridge which includes OpenCV4 which uses _Atomic
// We want to ignore this warning entirely.
#if defined(__clang__)
# pragma clang diagnostic push
#endif

#if defined(__clang__) && defined(__has_warning)
# if __has_warning( "-Wc11-extensions" )
#  pragma clang diagnostic ignored "-Wc11-extensions"
# endif
#endif

#if defined(__clang__)
# pragma clang diagnostic pop
#endif

// #define WITH_TIMING
#ifdef WITH_TIMING
#include <chrono>
#endif

int main( int argc, char** argv ){
  // Turn off warning message about labels
  // TODO: look into how labels are used
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


  ros::init(argc, argv, "plane_seg");
  ros::NodeHandle nh("~");
  std::unique_ptr<Pass> app = std::make_unique<Pass>(nh);

  ROS_INFO_STREAM("plane_seg ros ready");
  ROS_INFO_STREAM("=============================");

  bool run_test_program = false;
  nh.param("/plane_seg/run_test_program", run_test_program, false); 
  std::cout << "run_test_program: " << run_test_program << "\n";


  // Enable this to run the test programs
  if (run_test_program){
    std::cout << "Running test examples\n";
    app->processFromFile(0);
    app->processFromFile(1);
    app->processFromFile(2);
    app->processFromFile(3);
    // RACE examples don't work well
    //app->processFromFile(4);
    //app->processFromFile(5);

    std::cout << "Finished!\n";
    exit(-1);
  }

  ROS_INFO_STREAM("Waiting for ROS messages");
  ros::spin();

  return 1;
}
