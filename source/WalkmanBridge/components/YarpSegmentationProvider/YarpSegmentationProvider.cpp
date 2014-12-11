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


using namespace armarx;


void YarpSegmentationProvider::reportNewPointCloudSegmentation(const ::Ice::Current&)
{
    ARMARX_LOG << "New point cloud segementation" << flush;
}

void YarpSegmentationProvider::reportPointCloudSegmentation(const ::visionx::PointCloud::SegmentList&, const ::Ice::Current&)
{
    boost::mutex::scoped_lock(segmentationMutex);

    segmentation = environmentalPrimitiveSegment->getEnvironmentalPrimitives();
    ARMARX_INFO << "New segmentation reported (size: " << segmentation.size() << ") => Storing for later procession";

    // push to yarp

}

void YarpSegmentationProvider::onInitComponent()
{
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

