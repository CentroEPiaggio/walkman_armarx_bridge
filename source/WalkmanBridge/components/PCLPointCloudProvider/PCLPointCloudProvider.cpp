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

#include "PCLPointCloudProvider.h"


using namespace armarx;


void PCLPointCloudProvider::onInitCapturingPointCloudProvider()
{
    ros::init(argc, argv, "sub_pcl");
    point_cloud_topic_name = getProperty<std::string>("point_cloud_topic");
    setNumberPointClouds(1);
}


void PCLPointCloudProvider::onStartCapture()
{
    ros::Subscriber sub = nh.subscribe<PointCloud>(point_cloud_topic_name, 1, callback);
    ros::spin();
}


void PCLPointCloudProvider::onStopCapture()
{
    sub.deregister();
}


void PCLPointCloudProvider::onExitComponent()
{

}

PropertyDefinitionsPtr PCLPointCloudProvider::createPropertyDefinitions()
{
    return PropertyDefinitionsPtr(new PCLPointCloudProviderPropertyDefinitions(
                                      getConfigIdentifier()));
}

