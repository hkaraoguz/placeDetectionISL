#include "Utility.h"
Place::Place()
{
    id = -1;

}

Place::Place(int id)
{
    this->id = id;
    this->memberIds = cv::Mat::zeros(1,1,CV_16UC1);

}
void Place::calculateMeanInvariant()
{
    Mat wholeInvariants;

    for(uint i = 0; i < this->members.size(); i++)
    {
        if(i == 0)
        {
            wholeInvariants = members.at(i).invariants;


        }
        else
        {
            cv::hconcat(wholeInvariants,members.at(i).invariants,wholeInvariants);

        }



    }

    this->memberInvariants = wholeInvariants.clone();

   // Mat avg;

    cv::reduce(wholeInvariants,this->meanInvariant,1,CV_REDUCE_AVG);

    this->memberIds = cv::Mat::zeros(this->members.size(),1,CV_32SC1);

    for(uint i = 0; i < this->members.size(); i++)
    {
        this->memberIds.at<unsigned short>(i,0) = this->members[i].id;

    }

   // this->meanInvariant = avg;




}


