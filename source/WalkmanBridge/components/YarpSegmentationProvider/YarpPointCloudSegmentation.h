#include <yarp/os/Bottle.h>
#include <assert.h>
#include <vector>
namespace walkman
{
namespace yarp_armarx
{


class Point3D
{
public:
    
    yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle temp;
        temp.addDouble(x);
        temp.addDouble(y);
        temp.addDouble(z);
        assert(temp.size()==get_size());
        return temp;
    }

    void fromBottle(yarp::os::Bottle* temp)
    {
        if (temp->get(0).isNull())
            return;
        x = temp->get(0).asDouble();
        y = temp->get(1).asDouble();
        z = temp->get(2).asDouble(); 
        return;
    }

    static int get_size(){return 3;};
    
    float x=0.0;
    float y=0.0;
    float z=0.0;
};

class Segment
{
public:
    
    yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle temp;
        temp.addInt(active);
        temp.addInt(id);
        temp.addDouble(curvatureMean);
        temp.addDouble(curvatureStandardDeviation);
        temp.addDouble(curvatureMax);
        temp.addDouble(curvatureMin);
        yarp::os::Bottle& list= temp.addList();
        list.append(centroid().toBottle());
        yarp::os::Bottle& list1= temp.addList();
        list1.append(meanNormal().toBottle());
        yarp::os::Bottle& list2= temp.addList();
        list2.append(principalAxisI().toBottle());
        yarp::os::Bottle& list3= temp.addList();
        list3.append(principalAxisJ().toBottle());
        yarp::os::Bottle& list4= temp.addList();
        list4.append(principalAxisK().toBottle());
        temp.addDouble(extentI);
        temp.addDouble(extentJ);
        temp.addDouble(extentK);
        assert(temp.size()==get_size());
        return temp;
    }
    
    void fromBottle(yarp::os::Bottle* temp)
    {
        int counter=0;
        if (temp->get(counter).isNull())
            return;
        if (temp->size()!=get_size())
            return;
        active=temp->get(counter++).asInt();
        id=temp->get(counter++).asInt();
        curvatureMean=temp->get(counter++).asDouble();
        curvatureStandardDeviation=temp->get(counter++).asDouble();
        curvatureMax=temp->get(counter++).asDouble();
        curvatureMin=temp->get(counter++).asDouble();
        for (int i=0;i<5;i++)
        {
            if (temp->get(counter).asList()==NULL)
                return;
            if (temp->get(counter).asList()->size()!=Point3D::get_size())
                return;
            surface[i].fromBottle(temp->get(counter).asList());
            counter++;
        }
        extentI=temp->get(counter++).asDouble();
        extentJ=temp->get(counter++).asDouble();
        extentK=temp->get(counter++).asDouble();
        return;
    }

    static int get_size(){return 14;};
    
    bool active=false;
    int id=-1;
    float curvatureMean=0.0;
    float curvatureStandardDeviation=0.0;
    float curvatureMax=0.0;
    float curvatureMin=0.0;
    ::walkman::yarp_armarx::Point3D& centroid(){return surface[0];}
    ::walkman::yarp_armarx::Point3D& meanNormal(){return surface[1];}
    ::walkman::yarp_armarx::Point3D& principalAxisI(){return surface[2];}
    ::walkman::yarp_armarx::Point3D& principalAxisJ(){return surface[3];}
    ::walkman::yarp_armarx::Point3D& principalAxisK(){return surface[4];}
    const ::walkman::yarp_armarx::Point3D& centroid()const{return surface[0];}
    const ::walkman::yarp_armarx::Point3D& meanNormal()const{return surface[1];}
    const ::walkman::yarp_armarx::Point3D& principalAxisI()const{return surface[2];}
    const ::walkman::yarp_armarx::Point3D& principalAxisJ()const{return surface[3];}
    const ::walkman::yarp_armarx::Point3D& principalAxisK()const{return surface[4];}
    float extentI=0.0;
    float extentJ=0.0;
    float extentK=0.0;
//     ::walkman::yarp_armarx::BoundingBox3D boundingBox;
//     ::walkman::yarp_armarx::Point3DList points;
//     ::walkman::yarp_armarx::Polytope3D polytope;
private:
    ::walkman::yarp_armarx::Point3D surface[5];
};

class SegmentList
{
public:
    yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle temp;
  
        temp.addInt(segments.size());
        for (auto seg:segments)
        {
            yarp::os::Bottle& list = temp.addList();
            list.append(seg.toBottle());
        }

        return temp;
    }
    
    void fromBottle(yarp::os::Bottle* temp)
    {
        if (temp->get(0).isNull())
        {
            return;
        }
        
        int counter=0;
        segments.resize(temp->get(counter++).asInt());
        for (unsigned int i=0;i<segments.size();i++)
        {
            if (temp->get(counter).asList()==NULL)
                return;
            if (temp->get(counter).asList()->size()!=Segment::get_size())
                return;
            segments[i].fromBottle(temp->get(counter++).asList());
        }
        
        return;
    }
    
    std::vector< ::walkman::yarp_armarx::Segment> segments;
};


// typedef ::std::vector< ::walkman::yarp_armarx::Point3D> Point3DList;


// struct Plane3D
// {
//     ::walkman::yarp_armarx::Point3D normal;
//     float hesseDistance;
// };

// struct PartitionPlane
// {
//     int id;
//     float area;
//     ::walkman::yarp_armarx::Plane3D plane;
//     ::walkman::yarp_armarx::Point3DList polygon;
//     ::walkman::yarp_armarx::Point3D polygonCenter;
// };

// typedef ::std::vector< ::walkman::yarp_armarx::PartitionPlane> PolytopeBoundaries;

// struct Polytope3D
// {
//     ::walkman::yarp_armarx::Point3D centroid;
//     ::walkman::yarp_armarx::PolytopeBoundaries boundaries;
// };


    }
}