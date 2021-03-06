/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    WalkmanBridge::ArmarXObjects::PCLPointCloudProvider
 * @author     Markus Grotz ( markus dot grotz at kit dot edu )
 * @date       2014
 * @copyright  http://www.gnu.org/licenses/gpl.txt
 *             GNU General Public License
 */

#ifndef _ARMARX_COMPONENT_WalkmanBridge_PCLPointCloudProvider_H
#define _ARMARX_COMPONENT_WalkmanBridge_PCLPointCloudProvider_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <Core/core/services/tasks/PeriodicTask.h>

#include <Core/core/Component.h>
#include <VisionX/core/CapturingPointCloudProvider.h>

namespace visionx {
  namespace PointCloud {
    /**
     * @class PCLPointCloudProviderPropertyDefinitions
     * @brief
     * @ingroup Components
     */
    class PCLPointCloudProviderPropertyDefinitions:
        public armarx::ComponentPropertyDefinitions
    {
    public:
	PCLPointCloudProviderPropertyDefinitions(std::string prefix):
	    ComponentPropertyDefinitions(prefix)
	{
	    defineOptionalProperty<Grid2DDimensions>("Dimensions", Grid2DDimensions(640,  480), "PointCloud Dimensions")
				    .setCaseInsensitive(true)
				    .map("320x240",     Grid2DDimensions(320,  240))
				    .map("640x480",     Grid2DDimensions(640,  480));
				    
	    defineOptionalProperty<std::string>("pointCloudROSTopic", "/camera/depth_registered/points", "the ROS topic to connect");
	}
    };

    /**
     * @class PCLPointCloudProvider
     * @brief A brief description
     *
     * Detailed Description
     */
    class PCLPointCloudProvider :
        virtual public CapturingPointCloudProvider
    {
    public:
        /**
         * @see armarx::ManagedIceObject::getDefaultName()
         */
        virtual std::string getDefaultName() const
        {
            return "PCLPointCloudProvider";
        }

    protected:
        /**
         * @see visionx::PointCloud::CapturingPointCloudProvider
         */
        virtual void onInitCapturingPointCloudProvider();

        /**
         * @see visionx::PointCloud::CapturingPointCloudProvider::
         */
        virtual void onExitCapturingPointCloudProvider();

        /**
         * @see visionx::PointCloud::CapturingPointCloudProvider::
         */
        virtual void onStartCapture(float frameRate,const Grid2DDimensions& dimensions);

        /**
         * @see visionx::PointCloud::CapturingPointCloudProvider::
         */
        virtual void onStopCapture();

        /**
         * @see visionx::PointCloud::CapturingPointCloudProvider::capture()
         */
        virtual bool capture(void** pointCloudBuffers);

        /**
         * @see PropertyUser::createPropertyDefinitions()
         */
        virtual armarx::PropertyDefinitionsPtr createPropertyDefinitions();

    private:

        bool convertPointCloud(sensor_msgs::PointCloud2ConstPtr&, void** pointCloudBuffers);
	
        void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
	
	void doROSSpinning();
	
	armarx::PeriodicTask<PCLPointCloudProvider>* spinningTask = 0;

        boost::shared_mutex pointCloudMutex;
	
	sensor_msgs::PointCloud2ConstPtr msg;
	
	Grid2DDimensions pointCloudDimensions;

        std::string point_cloud_topic_name;

        ros::NodeHandle nh;

        ros::Subscriber sub;

	bool is_latest_msg;

    };
}

}


#endif
