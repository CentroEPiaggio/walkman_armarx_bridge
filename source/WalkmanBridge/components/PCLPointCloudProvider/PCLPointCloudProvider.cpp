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


namespace visionx {
  namespace PointCloud {
      
void PCLPointCloudProvider::onInitCapturingPointCloudProvider()
{
    point_cloud_topic_name = getProperty<std::string>("pointCloudROSTopic").getValue();

    setNumberPointClouds(1);


//    exampleTask = new RunningTask<RunningTaskExample>(this, &RunningTaskExample::exampleThreadMethod);

    //;== while(1) ros_spin_once();

}


void PCLPointCloudProvider::onExitCapturingPointCloudProvider()
{

}


//void exampleThreadMethod()
//    {
//        // do the work
//        printf("Thread method\n");

//        // shutdown is set if exampleTask->stop() is called.
//        // running tasks need to handle the shutdown by themselves.
//        // Here it is essentially not necessary but just serves as an example
//        while(!exampleTask->isStopped())
//        {
//            usleep(10);
//        }
//    }



void PCLPointCloudProvider::onStartCapture(float frameRate, const Grid2DDimensions& dimensions)
{
    sub = nh.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_name, 1, &PCLPointCloudProvider::callback,this);
}


void PCLPointCloudProvider::onStopCapture()
{
//TODO
  
  sub.shutdown();
}


void PCLPointCloudProvider::callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
   boost::mutex::scoped_lock(pointCloudMutex);

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputPCLCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::fromROSMsg(*msg,*InputPCLCloud);

   visionx::PointCloud::PointCloudFormatInfo Information;
   Information.dimensions.width = InputPCLCloud->width;
   Information.dimensions.height = InputPCLCloud->height;
   Information.bytesPerPoint = sizeof(visionx::PointCloud::RGBA) + sizeof(visionx::PointCloud::Point3D);
   Information.pointCloudStructure = visionx::PointCloud::PointCloudStructureType::eStructured;
   Information.totalVisiblePoints = -1;
   Information.pointContent = visionx::PointCloud::eCbrCbgCbbCba_PfxPfyPfz;

   pcl::PointXYZRGB* pPCLSource = const_cast<pcl::PointXYZRGB*>(&InputPCLCloud->points[0]);


   const int Area = Information.dimensions.width * Information.dimensions.height;
   // store this somewhere else
   visionx::PointCloud::ColoredPoint3D* pBufferBase = new visionx::PointCloud::ColoredPoint3D[Area];
   visionx::PointCloud::ColoredPoint3D* pBuffer = pBufferBase;
   visionx::PointCloud::ColoredPoint3D* pEndBuffer = pBuffer + Area;
   while(pBuffer<pEndBuffer)
   {
       pBuffer->color.r = pPCLSource->r;
       pBuffer->color.g = pPCLSource->g;
       pBuffer->color.b = pPCLSource->b;
       pBuffer->point.x = pPCLSource->x;
       pBuffer->point.y = pPCLSource->y;
       pBuffer->point.z = pPCLSource->z;

       //memcpy(&pBuffer->color,&pPCLSource->r,size(uint8_t)*3);
       //memcpy(&pBuffer->point,pPCLSource->data,size(float)*3);

       ++pBuffer;
       ++pPCLSource;
   }

}



bool PCLPointCloudProvider::capture(void** pointCloudBuffers)
{
   boost::mutex::scoped_lock(pointCloudMutex);

//    pointCloudBuffers = map_point_cloud(pointCloud);
   return true;
}



PropertyDefinitionsPtr PCLPointCloudProvider::createPropertyDefinitions()
{
    return PropertyDefinitionsPtr(new PCLPointCloudProviderPropertyDefinitions(
                                      getConfigIdentifier()));
}

    
    
  }
  
}
