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

#ifndef _ARMARX_COMPONENT_WalkmanBridge_YarpSegmentationProvider_H
#define _ARMARX_COMPONENT_WalkmanBridge_YarpSegmentationProvider_H


#include <Core/core/Component.h>

#include <VisionX/interface/component/PointCloudSegmentation.h>

#include <MemoryX/interface/component/WorkingMemoryInterface.h>
#include <MemoryX/core/memory/SegmentedMemory.h>
#include <MemoryX/libraries/memorytypes/segment/EnvironmentalPrimitiveSegment.h>

#include <yarp/os/all.h>

namespace armarx
{
    /**
     * @class YarpSegmentationProviderPropertyDefinitions
     * @brief
     * @ingroup Components
     */
    class YarpSegmentationProviderPropertyDefinitions:
        public ComponentPropertyDefinitions
    {
    public:
        YarpSegmentationProviderPropertyDefinitions(std::string prefix):
            ComponentPropertyDefinitions(prefix)
        {
            //defineRequiredProperty<std::string>("PropertyName", "Description");
            //defineOptionalProperty<std::string>("PropertyName", "DefaultValue", "Description");
	   defineOptionalProperty<std::string>("WorkingMemoryName", "WorkingMemory", "Name of WorkingMemory component");
        }
    };

    /**
     * @class YarpSegmentationProvider
     * @brief A brief description
     *
     * Detailed Description
     */
    class YarpSegmentationProvider :
        virtual public armarx::Component,
        virtual public visionx::PointCloud::PointCloudSegmentationListener
    {
    public:
        /**
         * @see armarx::ManagedIceObject::getDefaultName()
         */
        virtual std::string getDefaultName() const
        {
            return "YarpSegmentationProvider";
        }

        virtual void reportNewPointCloudSegmentation(const ::Ice::Current& = ::Ice::Current());

        virtual void reportPointCloudSegmentation(const ::visionx::PointCloud::SegmentList&, const ::Ice::Current& = ::Ice::Current());

    protected:
        /**
         * @see armarx::ManagedIceObject::onInitComponent()
         */
        virtual void onInitComponent();

        /**
         * @see armarx::ManagedIceObject::onConnectComponent()
         */
        virtual void onConnectComponent();

        /**
         * @see armarx::ManagedIceObject::onDisconnectComponent()
         */
        virtual void onDisconnectComponent();

        /**
         * @see armarx::ManagedIceObject::onExitComponent()
         */
        virtual void onExitComponent();

        /**
         * @see PropertyUser::createPropertyDefinitions()
         */
        virtual PropertyDefinitionsPtr createPropertyDefinitions();

    private:

         Mutex segmentationMutex;

         memoryx::WorkingMemoryInterfacePrx workingMemoryPrx;

         memoryx::EnvironmentalPrimitiveSegmentBasePrx environmentalPrimitiveSegment;

         memoryx::EnvironmentalPrimitiveBaseList segmentation;

	 yarp::os::BufferedPort<yarp::os::Bottle> output_port;
    };
}

#endif
