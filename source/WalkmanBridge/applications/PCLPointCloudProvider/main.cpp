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
 * @package    WalkmanBridge::application::PCLPointCloudProvider
 * @author     Markus Grotz ( markus dot grotz at kit dot edu )
 * @date       2014
 * @copyright  http://www.gnu.org/licenses/gpl.txt
 *             GNU General Public License
 */

#include "PCLPointCloudProviderApp.h"
#include <Core/core/logging/Logging.h>

#include <ros/ros.h>

int main(int argc, char* argv[])
{
    armarx::ApplicationPtr app = armarx::Application::createInstance < armarx::PCLPointCloudProviderApp > ();
    app->setName("PCLPointCloudProvider");

    ros::init(argc, argv, "armarx_walkman_brige_pcl");

    return app->main(argc, argv);
}
