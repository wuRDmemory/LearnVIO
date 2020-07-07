#include "../../include/visual/visual_init.h"

// void normalizePoints(const vector<Point2f> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T) {
//     float meanX = 0;
//     float meanY = 0;
//     const int N = vKeys.size();

//     vNormalizedPoints.resize(N);

//     for(int i=0; i<N; i++)
//     {
//         meanX += vKeys[i].x;
//         meanY += vKeys[i].y;
//     }

//     meanX = meanX/N;
//     meanY = meanY/N;

//     float meanDevX = 0;
//     float meanDevY = 0;

//     for(int i=0; i<N; i++)
//     {
//         vNormalizedPoints[i].x = vKeys[i].x - meanX;
//         vNormalizedPoints[i].y = vKeys[i].y - meanY;

//         meanDevX += fabs(vNormalizedPoints[i].x);
//         meanDevY += fabs(vNormalizedPoints[i].y);
//     }

//     meanDevX = meanDevX/N;
//     meanDevY = meanDevY/N;

//     float sX = 1.0/meanDevX;
//     float sY = 1.0/meanDevY;

//     for(int i=0; i<N; i++)
//     {
//         vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
//         vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
//     }

//     T = cv::Mat::eye(3,3,CV_64F);
//     T.at<double>(0,0) = sX;
//     T.at<double>(1,1) = sY;
//     T.at<double>(0,2) = -meanX*sX;
//     T.at<double>(1,2) = -meanY*sY;
// }

// cv::Mat computeE21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2) {
//     const int N = vP1.size();

//     cv::Mat A(N,9,CV_32F);

//     for(int i=0; i<N; i++)
//     {
//         const float u1 = vP1[i].x;
//         const float v1 = vP1[i].y;
//         const float u2 = vP2[i].x;
//         const float v2 = vP2[i].y;

//         A.at<float>(i,0) = u2*u1;
//         A.at<float>(i,1) = u2*v1;
//         A.at<float>(i,2) = u2;
//         A.at<float>(i,3) = v2*u1;
//         A.at<float>(i,4) = v2*v1;
//         A.at<float>(i,5) = v2;
//         A.at<float>(i,6) = u1;
//         A.at<float>(i,7) = v1;
//         A.at<float>(i,8) = 1;
//     }

//     cv::Mat u,w,vt;

//     cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

//     cv::Mat Fpre = vt.row(8).reshape(0, 3);

//     cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

//     w.at<float>(2)=0;

//     return  u*cv::Mat::diag(w)*vt;
// }

// void decomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
// {
//     cv::Mat u,w,vt;
//     cv::SVD::compute(E,w,u,vt);

//     u.col(2).copyTo(t);
//     t=t/cv::norm(t);

//     cv::Mat W(3,3,CV_32F,cv::Scalar(0));
//     W.at<float>(0,1)=-1;
//     W.at<float>(1,0)=1;
//     W.at<float>(2,2)=1;

//     R1 = u*W*vt;
//     if(cv::determinant(R1)<0)
//         R1=-R1;

//     R2 = u*W.t()*vt;
//     if(cv::determinant(R2)<0)
//         R2=-R2;
// }

// float checkEssential(const cv::Mat &F21, const vector<Point2f> &pts1, 
//                        const vector<Point2f> &pts2, vector<bool> &vbMatchesInliers, float sigma) {
//     const int N = pts1.size();

//     const float f11 = F21.at<float>(0,0);
//     const float f12 = F21.at<float>(0,1);
//     const float f13 = F21.at<float>(0,2);
//     const float f21 = F21.at<float>(1,0);
//     const float f22 = F21.at<float>(1,1);
//     const float f23 = F21.at<float>(1,2);
//     const float f31 = F21.at<float>(2,0);
//     const float f32 = F21.at<float>(2,1);
//     const float f33 = F21.at<float>(2,2);

//     vbMatchesInliers.resize(N);

//     float score = 0;

//     const float th = 3.841;
//     const float thScore = 5.991;

//     const float invSigmaSquare = 1.0/(sigma*sigma);

//     for(int i=0; i<N; i++)
//     {
//         bool bIn = true;

//         const cv::Point2f &kp1 = pts1[i];
//         const cv::Point2f &kp2 = pts2[i];

//         const float u1 = kp1.pt.x;
//         const float v1 = kp1.pt.y;
//         const float u2 = kp2.pt.x;
//         const float v2 = kp2.pt.y;

//         // Reprojection error in second image
//         // l2=F21x1=(a2,b2,c2)

//         const float a2 = f11*u1+f12*v1+f13;
//         const float b2 = f21*u1+f22*v1+f23;
//         const float c2 = f31*u1+f32*v1+f33;

//         const float num2 = a2*u2+b2*v2+c2;

//         const float squareDist1 = num2*num2/(a2*a2+b2*b2);

//         const float chiSquare1 = squareDist1*invSigmaSquare;

//         if(chiSquare1>th)
//             bIn = false;
//         else
//             score += thScore - chiSquare1;

//         // Reprojection error in second image
//         // l1 =x2tF21=(a1,b1,c1)

//         const float a1 = f11*u2+f21*v2+f31;
//         const float b1 = f12*u2+f22*v2+f32;
//         const float c1 = f13*u2+f23*v2+f33;

//         const float num1 = a1*u1+b1*v1+c1;

//         const float squareDist2 = num1*num1/(a1*a1+b1*b1);

//         const float chiSquare2 = squareDist2*invSigmaSquare;

//         if(chiSquare2>th)
//             bIn = false;
//         else
//             score += thScore - chiSquare2;

//         if(bIn)
//             vbMatchesInliers[i]=true;
//         else
//             vbMatchesInliers[i]=false;
//     }

//     return score;
// }

// bool reconstructF(vector<bool> &vbMatchesInliers, cv::Mat &E21,
//                   cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
// {
//     int N=0;
//     for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
//         if(vbMatchesInliers[i])
//             N++;

//     cv::Mat R1, R2, t;

//     // Recover the 4 motion hypotheses
//     decomposeE(E21,R1,R2,t);  

//     cv::Mat t1=t;
//     cv::Mat t2=-t;

//     // Reconstruct with the 4 hyphoteses and check
//     vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
//     vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
//     float parallax1,parallax2, parallax3, parallax4;

//     int nGood1 = checkRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
//     int nGood2 = checkRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
//     int nGood3 = checkRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
//     int nGood4 = checkRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

//     int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

//     R21 = cv::Mat();
//     t21 = cv::Mat();

//     int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

//     int nsimilar = 0;
//     if(nGood1>0.7*maxGood)
//         nsimilar++;
//     if(nGood2>0.7*maxGood)
//         nsimilar++;
//     if(nGood3>0.7*maxGood)
//         nsimilar++;
//     if(nGood4>0.7*maxGood)
//         nsimilar++;

//     // If there is not a clear winner or not enough triangulated points reject initialization
//     if(maxGood<nMinGood || nsimilar>1)
//     {
//         return false;
//     }

//     // If best reconstruction has enough parallax initialize
//     if(maxGood==nGood1)
//     {
//         if(parallax1>minParallax)
//         {
//             vP3D = vP3D1;
//             vbTriangulated = vbTriangulated1;

//             R1.copyTo(R21);
//             t1.copyTo(t21);
//             return true;
//         }
//     }else if(maxGood==nGood2)
//     {
//         if(parallax2>minParallax)
//         {
//             vP3D = vP3D2;
//             vbTriangulated = vbTriangulated2;

//             R2.copyTo(R21);
//             t1.copyTo(t21);
//             return true;
//         }
//     }else if(maxGood==nGood3)
//     {
//         if(parallax3>minParallax)
//         {
//             vP3D = vP3D3;
//             vbTriangulated = vbTriangulated3;

//             R1.copyTo(R21);
//             t2.copyTo(t21);
//             return true;
//         }
//     }else if(maxGood==nGood4)
//     {
//         if(parallax4>minParallax)
//         {
//             vP3D = vP3D4;
//             vbTriangulated = vbTriangulated4;

//             R2.copyTo(R21);
//             t2.copyTo(t21);
//             return true;
//         }
//     }

//     return false;
// }

void findEssential(const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, cv::Mat &E21) {

}
