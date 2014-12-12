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
 * @package    WalkmanBridge::ArmarXObjects::YarpSegmentationProvider
 * @author     Markus Grotz ( markus dot grotz at kit dot edu )
 * @date       2014
 * @copyright  http://www.gnu.org/licenses/gpl.txt
 *             GNU General Public License
 */

#include "YarpSegmentationProvider.h"
#include "YarpPointCloudSegmentation.h"

using namespace armarx;

Vector3BasePtr generatePoint()
{
    Vector3BasePtr point(new Vector3);

    point->x=3+((double)(rand()%10))/10.0;
    point->y=4+((double)(rand()%10))/10.0;
    point->z=5+((double)(rand()%10))/10.0;
    return point;
}


memoryx::EnvironmentalPrimitiveBasePtr generateSegment()
{
    memoryx::EnvironmentalPrimitiveBasePtr segment(new memoryx::EnvironmentalPrimitive);
    segment->setActive(true);
    segment->setCentroid(generatePoint());
    segment->setMaxCurvature(((double)(rand()%10)));
    segment->setMinCurvature(((double)(rand()%10)));
    segment->setMeanCurvature(((double)(rand()%10)));
    segment->setStandardDeviation(((double)(rand()%10)));
    segment->setExtentI(((double)(rand()%10)));
    segment->setExtentJ(((double)(rand()%10)));
    segment->setExtentK(((double)(rand()%10)));
    segment->setMeanNormal(generatePoint());
    segment->setPrincipalAxisI(generatePoint());
    segment->setPrincipalAxisJ(generatePoint());
    segment->setPrincipalAxisK(generatePoint());
    return segment;
}

void YarpSegmentationProvider::reportPointCloudSegmentation(const ::visionx::PointCloud::SegmentList&, const ::Ice::Current&)
{
    ARMARX_LOG << "New point cloud segmentation" << flush;
}

void YarpSegmentationProvider::reportNewPointCloudSegmentation(const ::Ice::Current&)
{
    boost::mutex::scoped_lock(segmentationMutex);

    //segmentation = environmentalPrimitiveSegment->getEnvironmentalPrimitives(); //NOTE: uncomment to process segments
    
    //NOTE: using fake segments
    std::cout<<" - Warning: publishing fakes segment -"<<std::endl;
    segmentation.clear();
    segmentation.push_back(generateSegment());
    segmentation.push_back(generateSegment());
    segmentation.push_back(generateSegment());
    
    ARMARX_INFO << "New segmentation reported (size: " << segmentation.size() << ") => Storing for later procession";
    walkman::yarp_armarx::SegmentList segment_list;

    for(auto segment:segmentation)
    {
	walkman::yarp_armarx::Segment temp_seg;
	temp_seg.active=segment->getActive();
	temp_seg.curvatureMean=segment->getMeanCurvature();
	temp_seg.curvatureStandardDeviation=segment->getStandardDeviation();
	temp_seg.curvatureMax=segment->getMaxCurvature();
	temp_seg.curvatureMin=segment->getMinCurvature();
	temp_seg.centroid().x=segment->getCentroid()->x;
	temp_seg.centroid().y=segment->getCentroid()->y;
	temp_seg.centroid().z=segment->getCentroid()->z;
	temp_seg.meanNormal().x=segment->getMeanNormal()->x;
	temp_seg.meanNormal().y=segment->getMeanNormal()->y;
	temp_seg.meanNormal().z=segment->getMeanNormal()->z;
	temp_seg.principalAxisI().x=segment->getPrincipalAxisI()->x;
	temp_seg.principalAxisI().y=segment->getPrincipalAxisI()->y;
	temp_seg.principalAxisI().z=segment->getPrincipalAxisI()->z;
	temp_seg.principalAxisJ().x=segment->getPrincipalAxisJ()->x;
	temp_seg.principalAxisJ().y=segment->getPrincipalAxisJ()->y;
	temp_seg.principalAxisJ().z=segment->getPrincipalAxisJ()->z;
	temp_seg.principalAxisK().x=segment->getPrincipalAxisK()->x;
	temp_seg.principalAxisK().y=segment->getPrincipalAxisK()->y;
	temp_seg.principalAxisK().z=segment->getPrincipalAxisK()->z;
	temp_seg.extentI=segment->getExtentI();
	temp_seg.extentJ=segment->getExtentJ();
	temp_seg.extentK=segment->getExtentK();

	segment_list.segments.push_back(temp_seg);
    }

    yarp::os::Bottle& to_write = output_port.prepare();
    to_write.clear();
    to_write.append(segment_list.toBottle());
    output_port.write();
}

void YarpSegmentationProvider::onInitComponent()
{
    std::string port_name_out="/yarp_armarx/segment:o";
    std::string port_name_in="/yarp_armarx/segment:i";

    output_port.open(port_name_out);

    if (!yarp::os::NetworkBase::isConnected(port_name_out,port_name_in))
    {
	yarp::os::ContactStyle style;
	style.persistent=true;
	std::cout<<"connecting "<<port_name_out <<" to " <<port_name_in<<std::endl;
	yarp::os::Network::connect(port_name_out,port_name_in,style);
    }
    
    usingProxy(getProperty<std::string>("WorkingMemoryName").getValue());

    usingTopic("SegmentedPointCloudScan");
}


void YarpSegmentationProvider::onConnectComponent()
{

    workingMemoryPrx = getProxy<memoryx::WorkingMemoryInterfacePrx>(getProperty<std::string>("WorkingMemoryName").getValue());
    if(!workingMemoryPrx)
    {
        ARMARX_ERROR << "Failed to obtain working memory proxy";
        return;
    }

    environmentalPrimitiveSegment = workingMemoryPrx->getEnvironmentalPrimitiveSegment();
    if(!environmentalPrimitiveSegment)
    {
        ARMARX_ERROR << "Failed to obtain environmental primitive segment pointer";
        return;
    }

}


void YarpSegmentationProvider::onDisconnectComponent()
{

}


void YarpSegmentationProvider::onExitComponent()
{

}

PropertyDefinitionsPtr YarpSegmentationProvider::createPropertyDefinitions()
{
    return PropertyDefinitionsPtr(new YarpSegmentationProviderPropertyDefinitions(
                                      getConfigIdentifier()));
}

