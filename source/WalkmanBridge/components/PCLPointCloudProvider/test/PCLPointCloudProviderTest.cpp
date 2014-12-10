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

#define BOOST_TEST_MODULE WalkmanBridge::ArmarXObjects::PCLPointCloudProvider

#define ARMARX_BOOST_TEST

#include <WalkmanBridge/Test.h>
#include <WalkmanBridge/components/PCLPointCloudProvider/PCLPointCloudProvider.h>

#include <iostream>

BOOST_AUTO_TEST_CASE(testExample)
{
    armarx::PCLPointCloudProvider instance;

    BOOST_CHECK_EQUAL(true, true);
}
