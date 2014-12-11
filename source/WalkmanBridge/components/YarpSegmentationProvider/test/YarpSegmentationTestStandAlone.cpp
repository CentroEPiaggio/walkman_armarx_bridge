
#include <WalkmanBridge/components/YarpSegmentationProvider/YarpPointCloudSegmentation.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include <yarp/os/all.h>
#include <assert.h>
#include <unistd.h>
using namespace walkman::yarp_armarx;


Point3D generatePoint()
{
    Point3D point;
    point.x=3+((double)(rand()%10))/10.0;
    point.y=4+((double)(rand()%10))/10.0;
    point.z=5+((double)(rand()%10))/10.0;
    return point;
}


Segment generateSegment()
{
    Segment segment;
    segment.active=true;
    segment.centroid()=generatePoint();
    segment.curvatureMax=((double)(rand()%10));
    segment.curvatureMin=((double)(rand()%10));
    segment.curvatureMean=((double)(rand()%10));
    segment.curvatureStandardDeviation=((double)(rand()%10));
    segment.extentI=((double)(rand()%10));
    segment.extentJ=((double)(rand()%10));
    segment.extentK=((double)(rand()%10));
    segment.id=6;
    segment.meanNormal()=generatePoint();
    segment.principalAxisI()=generatePoint();
    segment.principalAxisJ()=generatePoint();
    segment.principalAxisK()=generatePoint();
    return segment;
}

void print_segment(const Segment& seg)
{
    std::cout<<std::endl;
    
    std::cout<<"active: "<<seg.active<<std::endl;
    std::cout<<"centroid: "<<seg.centroid().x<<' '<<seg.centroid().y<<' '<<seg.centroid().z<<std::endl;
    std::cout<<"curvatureMax: "<<seg.curvatureMax<<std::endl;
    std::cout<<"curvatureMin: "<<seg.curvatureMin<<std::endl;
    std::cout<<"curvatureMean: "<<seg.curvatureMean<<std::endl;
    std::cout<<"curvatureStandardDeviation: "<<seg.curvatureStandardDeviation<<std::endl;
    std::cout<<"extentI: "<<seg.extentI<<std::endl;
    std::cout<<"extentJ: "<<seg.extentJ<<std::endl;
    std::cout<<"extentK: "<<seg.extentK<<std::endl;
    std::cout<<"id: "<<seg.id<<std::endl;
    std::cout<<"meanNormal: "<<seg.meanNormal().x<<' '<<seg.meanNormal().y<<' '<<seg.meanNormal().z<<std::endl;
    std::cout<<"principalAxisI: "<<seg.principalAxisI().x<<' '<<seg.principalAxisI().y<<' '<<seg.principalAxisI().z<<std::endl;
    std::cout<<"principalAxisJ: "<<seg.principalAxisJ().x<<' '<<seg.principalAxisJ().y<<' '<<seg.principalAxisJ().z<<std::endl;
    std::cout<<"principalAxisK: "<<seg.principalAxisK().x<<' '<<seg.principalAxisK().y<<' '<<seg.principalAxisK().z<<std::endl;
    
    std::cout<<std::endl;
}

void print_segment(const SegmentList& seg_list)
{
    for(auto seg:seg_list.segments) print_segment(seg);
}

#define EQUAL3(X, Y, Z) X.Z==Y.Z
#define EQUAL5(X, Y, Z, W, T) (EQUAL3(X,Y,Z))&&(EQUAL3(X,Y,W))&&(EQUAL3(X,Y,T))

int main(int argc, char* argv[])
{
    /* initialize random seed: */
    srand (time(NULL));
    SegmentList initial,final;
    for (int i=0;i<6;i++)
    {
        initial.segments.push_back(generateSegment());
    }
    // yarp network declaration and check
//     print_segment(initial);
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running, using local serialization"<< std::endl;
        yarp::os::Bottle temp=initial.toBottle();
        final.fromBottle(&temp);
    }
    else
    {
        // yarp network initialization
        yarp.init();
        yarp::os::BufferedPort<yarp::os::Bottle> port_out,port_in;
        std::string port_name_out="/test::out";
        std::string port_name_in="/test::in";
        
        bool result = port_out.open(port_name_out);
        result = port_in.open(port_name_in);
        
        if (!yarp::os::NetworkBase::isConnected(port_name_out,port_name_in))
        {
            yarp::os::ContactStyle style;
            style.persistent=true;
            std::cout<<"connecting "<<port_name_out <<" to " <<port_name_in<<std::endl;
            yarp::os::Network::connect(port_name_out,port_name_in,style);
        }

        yarp::os::Bottle& to_write = port_out.prepare();
        to_write.clear();
        to_write.append(initial.toBottle());
        port_out.write();
	sleep(1); //NOTE: the read after the write does not receive the message, you can copy from the terminal
        yarp::os::Bottle* temp=port_in.read(true);
        final.fromBottle(temp);
    }
//     print_segment(final);
    assert(initial.segments.size()==final.segments.size());
    for (int i=0;i<initial.segments.size();i++)
    {
        Segment& init=initial.segments[i];
        Segment& fin=final.segments[i];
        assert(EQUAL3(init,fin,active));
        assert(EQUAL3(init,fin,id));
        assert(EQUAL3(init,fin,curvatureMin));
        assert(EQUAL3(init,fin,curvatureMax));
        assert(EQUAL3(init,fin,curvatureMean));
        assert(EQUAL3(init,fin,curvatureStandardDeviation));
        assert(EQUAL3(init,fin,extentI));
        assert(EQUAL3(init,fin,extentJ));
        assert(EQUAL3(init,fin,extentK));
        assert(EQUAL5(init.centroid(),fin.centroid(),x,y,z));
        assert(EQUAL5(init.meanNormal(),fin.meanNormal(),x,y,z));
        assert(EQUAL5(init.principalAxisI(),fin.principalAxisI(),x,y,z));
        assert(EQUAL5(init.principalAxisJ(),fin.principalAxisJ(),x,y,z));
        assert(EQUAL5(init.principalAxisK(),fin.principalAxisK(),x,y,z));
    }
    std::cout<<"Everything ok"<<std::endl;
    
}














