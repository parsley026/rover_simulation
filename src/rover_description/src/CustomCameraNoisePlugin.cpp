#include <memory>
#include <random>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/common/Console.hh>

namespace rover_description
{
  class CustomCameraNoisePlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure
  {
  public:
    void Configure(const gz::sim::Entity &/*_entity*/,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &/*_ecm*/,
                   gz::sim::EventManager &/*_eventMgr*/) override
    {
      // 1. Read custom parameters
      if (_sdf->HasElement("param1"))
        this->param1 = _sdf->Get<double>("param1");
      
      if (_sdf->HasElement("param2"))
        this->param2 = _sdf->Get<double>("param2");

      // 2. Read the camera topic directly from the plugin configuration
      std::string cameraTopic;
      if (_sdf->HasElement("topic")) {
        cameraTopic = _sdf->Get<std::string>("topic");
      } else {
        gzerr << "[CustomCameraNoisePlugin] <topic> not specified in plugin! Cannot apply noise." << std::endl;
        return;
      }
        
      std::string noisyTopic = cameraTopic + "/noisy";

      // 3. Setup Gazebo Transport publishers and subscribers
      gz::transport::AdvertiseMessageOptions opts;
      opts.SetMsgsPerSec(30); 
      this->pub = this->node.Advertise<gz::msgs::Image>(noisyTopic, opts);
      
      // Subscribe to original camera
      this->node.Subscribe(cameraTopic, &CustomCameraNoisePlugin::OnImage, this);

      gzmsg << "[CustomCameraNoisePlugin] Configured with param1=" 
            << this->param1 << ", param2=" << this->param2 << std::endl;
      gzmsg << "[CustomCameraNoisePlugin] Listening to: " << cameraTopic << std::endl;
      gzmsg << "[CustomCameraNoisePlugin] Publishing to: " << noisyTopic << std::endl;
    }

  private:
void OnImage(const gz::msgs::Image &_msg)
    {
      gz::msgs::Image noisy_msg = _msg;
      int channels = 0;

      // Determine bytes per pixel based on the Gazebo image format
      if (_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8 || 
          _msg.pixel_format_type() == gz::msgs::PixelFormatType::BGR_INT8) {
        channels = 3;
      } else if (_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
        channels = 1; // Left/Right Stereo cameras are usually grayscale
      }

      if (channels > 0) 
      {
        std::string *data = noisy_msg.mutable_data();
        uint8_t *pixels = reinterpret_cast<uint8_t*>(&(*data)[0]);
        
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        // Advance by the correct number of channels to avoid memory corruption
        for (size_t i = 0; i < data->size(); i += channels)
        {
          double rand_val = dis(gen);
          
          if (rand_val < this->param2) { 
            // Salt (White pixel)
            pixels[i] = 255;   
            if (channels == 3) { pixels[i+1] = 255; pixels[i+2] = 255; }
          } else if (rand_val > 1.0 - this->param2) {
            // Pepper (Black pixel)
            pixels[i] = 0;
            if (channels == 3) { pixels[i+1] = 0; pixels[i+2] = 0; }
          }
        }
      } 
      else 
      {
        // Warn if it's a 16-bit depth image or something completely unhandled
        static bool warned = false;
        if (!warned) {
          gzerr << "[CustomCameraNoisePlugin] FORMAT MISMATCH! Received format type: " 
                << _msg.pixel_format_type() 
                << ". Bypassing noise loop. Image will be clean." << std::endl;
          warned = true;
        }
      }

      this->pub.Publish(noisy_msg);
    }
    
    // Default values if missing from URDF
    double param1{0.5};
    double param2{0.05}; 
    
    gz::transport::Node node;
    gz::transport::Node::Publisher pub;
  };
}

// 4. Register the plugin so Gazebo sim can load it
GZ_ADD_PLUGIN(
  rover_description::CustomCameraNoisePlugin,
  gz::sim::System,
  rover_description::CustomCameraNoisePlugin::ISystemConfigure
)
GZ_ADD_PLUGIN_ALIAS(
  rover_description::CustomCameraNoisePlugin,
  "custom_camera_noise_plugin"
)