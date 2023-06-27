// #include "model.hpp"
// #include "ros/ros.h"
// #include "gazebo_msgs/ModelStates.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PointStamped.h"
// #include <cmath>
// #include <iostream>
// #include <vector>
// #include <random>

// // For paper 

// geometry_msgs::PoseStamped GOAL;
// geometry_msgs::PoseStamped curr_state;


// constexpr size_t p = 18; // размерность вектора параметров 
// constexpr size_t HH = 100; // количество возможных решений
// constexpr size_t PP = 1500; // 
// std::vector<std::vector<float>> psoP(HH, std::vector<float>(p)); // множество решений
// std::vector<std::vector<float>> psoV(HH, std::vector<float>(p)); // множество скоростей решений
// std::vector<float> qmin = {-1,-1,-1.57, -1,-1,-1.57, -1,-1,-1.57, -1,-1,-1.57, -1,-1,-1.57, -1,-1,-1.57};
// std::vector<float> qmax = {11,11,1.57, 11,11,1.57, 11,11,1.57, 11,11,1.57, 11,11,1.57, 11,11,1.57};
// std::vector<float>Fuh(HH);
// std::vector<float>qq(p);
// std::vector<float>q2(p);
// size_t n = 6; // ??
// size_t m = 4; // ?? 
// float t = 0;
// float dt = 0.01; // 
// float dtp = 0.1;

// int kR = 3; 
// int kP = 3; 
// int kV = 2;
// int L = 24;
// int rad_obc = 2; // радиус между объектами  // ob - объект
// float eps1 = 0.01;  // 
// float tf = 2.8; //  max time
// float alpha = 0.7298;
// float betta = 0.85;
// int infs = 5;
// float g_ = 0.1;

// std::vector<float> q1_c = {14.72876, 2.02710, 4.02222};
// std::vector<float> fb(n);
// std::vector<float> fa(n);
// std::vector<float> Vs(kP);
// std::vector<float> Cs = q1_c;
// std::vector<float> x01 ={0,0,0,10,10,0}; // ??? initial state
// std::vector<float> xfc = {10,10,0,0,0,0}; // ??? terminal state
// std::vector<int> Pnum = {0,1,2};
// std::vector<float> x = x01; // 
// std::vector<float>u(m, 0); // 
// std::vector<std::vector<float>> xy_prc = {{2.5,2.5},{1.5,8.5},{8.5,1.5},{7.5,7.5}}; // координаты центров препят
// std::vector<float> rad_prc = {2.5,3,3,2.5}; // радиусы препят
// std::vector<float> psoBestInform(p);
// std::vector<float> psoBestGlobal(p);
// std::vector<float> qnew(p);




// void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr &msg) {
//   // std::cout<< "model callback"<< std::endl;
//   const auto &q = msg->pose[1].orientation;

//   double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
//   double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
//   double yaw = std::atan2(siny_cosp, cosy_cosp);

//   curr_state.pose.position.x = msg->pose[1].position.x;
//   curr_state.pose.position.y = msg->pose[1].position.y;
//   curr_state.pose.orientation.x = q.x;
//   curr_state.pose.orientation.y = q.y;
//   curr_state.pose.orientation.z = q.z;
//   curr_state.pose.orientation.w = q.w;
// }




// void Initial()
// {
//   x = x01;
//   u = std::vector<float>(m, 0);
//   t =0;
//   Cs=q1_c;
// }


// float NormDist(int k, std::vector<float> x1, std::vector<float> xf1)
// {
//   float result = 0.;
  
//   for(size_t i = 0; i < 3; ++i)
//     result = std::max((xf1[i+k*3]-x1[i+k*3]), result);
  
//   return result;
// }


// void Upr(float& tt)
// {
//   float delt_t = 0.7;
//   if (tt<delt_t)
//   {
//     for (size_t i=0; i<n; ++i)
//     {
//       xf1[i]:=q2[i]
//     }
//   }
//   else
//   {
//     if (tt<delt_t*2)
//     {
//       for i:=0 to n-1 
//       {
//         xf1[i]:=q2[i+p2]
//       }
//     }
//     else{
//       if (tt < delt_t*3) {
//         for( size_t i=0; i < n; i++)
//         {
//           xf1[i]:=q2[i+2*p2]
//         }
//       }
//       else {
//         for( size_t i=0; i< n; ++i) 
//         {
//           xf1[i]:=xfc[i];
//         }
//       }
//     }
//   }
// }

// void Euler2(float& dt1, std::vector<float>& xx, float& tt){
//   Right_Part(tt,xx,fa);
//   for(size_t i=0; i<n; ++i)
//   {
//     x1[i]=xx[i]+dt1*fa[i];
//   }
//   Right_Part(tt+dt1,x1,fb);
//   for(size_t i=0; i<n-1; ++i){
//     xx[i]=xx[i]+dt1*(fa[i]+fb[i])/2;
//   }
//   tt=tt+dt1;
// }


// Procedure NOP_RPControl;
// var
//   i,j:integer;
//   zz:real;
// Begin
//   for i:=0 to L-1 do
//     case psi[i,i] of
//       1,5..8: z[i]:=0;
//       2: z[i]:=1;
//       3: z[i]:=-infinity;
//       4: z[i]:=infinity;
//     end;
//   for i:=0 to kP-1 do
//     z[Pnum[i]]:=Vs[i];
//   for i:=0 to kR-1 do
//     z[Rnum[i]]:=Cs[i];
//   for i:=0 to L-2 do
//     for j:=i+1 to L-1 do
//       if Psi[i,j]<>0 then
//       begin
//         case Psi[i,j] of
//           1: zz:=Ro_1(z[i]);
//           2: zz:=Ro_2(z[i]);
//           3: zz:=Ro_3(z[i]);
//           4: zz:=Ro_4(z[i]);
//           5: zz:=Ro_5(z[i]);
//           6: zz:=Ro_6(z[i]);
//           7: zz:=Ro_7(z[i]);
//           8: zz:=Ro_8(z[i]);
//           9: zz:=Ro_9(z[i]);
//           10: zz:=Ro_10(z[i]);
//           11: zz:=Ro_11(z[i]);
//           12: zz:=Ro_12(z[i]);
//           13: zz:=Ro_13(z[i]);
//           14: zz:=Ro_14(z[i]);
//           15: zz:=Ro_15(z[i]);
//           16: zz:=Ro_16(z[i]);
//           17: zz:=Ro_17(z[i]);
//           18: zz:=Ro_18(z[i]);
//           19: zz:=Ro_19(z[i]);
//           20: zz:=Ro_20(z[i]);
//           21: zz:=Ro_21(z[i]);
//           22: zz:=Ro_22(z[i]);
//           23: zz:=Ro_23(z[i]);
//           24: zz:=Ro_24(z[i]);
//         end;
//         case Psi[j,j] of
//           1: z[j]:=Xi_1(z[j],zz);
//           2: z[j]:=Xi_2(z[j],zz);
//           3: z[j]:=Xi_3(z[j],zz);
//           4: z[j]:=Xi_4(z[j],zz);
//           5: z[j]:=Xi_5(z[j],zz);
//           6: z[j]:=Xi_6(z[j],zz);
//           7: z[j]:=Xi_7(z[j],zz);
//           8: z[j]:=Xi_8(z[j],zz);
//         end;
//       end;
// End;



// //*************************************************************
// Procedure Right_Part(float& t1, std::vector<float>xx1, std::vector<float>& ff){
// Begin
//   Upr(t1);
//   Vs[PNum[0]]:=xf1[0]-xx1[0];
//   Vs[PNum[1]]:=xf1[1]-xx1[1];
//   Vs[PNum[2]]:=xf1[2]-xx1[2];
//   NOP_RPControl;
//   if Normdist(0,xx1,xf1)<eps1 then
//   begin
//     u[0]:=0;
//     u[1]:=0;
//   end
//   else
//   begin
//     u[0]:=z[DNum[0]];
//     u[1]:=z[DNum[1]];
//   end;
//   Vs[PNum[0]]:=xf1[3]-xx1[3];
//   Vs[PNum[1]]:=xf1[4]-xx1[4];
//   Vs[PNum[2]]:=xf1[5]-xx1[5];
//   NOP_RPControl;
//   if Normdist(1,xx1,xf1)<eps1 then
//   begin
//     u[2]:=0;
//     u[3]:=0;
//   end
//   else
//   begin
//     u[2]:=z[DNum[0]];
//     u[3]:=z[DNum[1]];
//   end;
//   OgrUpr;
//   ff[0]:=0.5*(u[0]+u[1])*cos(xx1[2]);
//   ff[1]:=0.5*(u[0]+u[1])*sin(xx1[2]);
//   ff[2]:=0.5*(u[0]-u[1]);
//   ff[3]:=0.5*(u[2]+u[3])*cos(xx1[5]);
//   ff[4]:=0.5*(u[2]+u[3])*sin(xx1[5]);
//   ff[5]:=0.5*(u[2]-u[3]);
// }



// float Goal(std::vector<float> qq1)
// {
//   q2 = qq1;
//   Initial();
//   float su = 0.;
//   t = 0.;
//   Euler2(dt,x,t);
//   float dr= NormDist(0,xfc,x);
//   float dr1 = NormDist(1,xfc,x);

//   auto step = [&](){
//   Euler2(dt,x,t);
//   float dr=NormDist(0,xfc,x);
//   float dr1 = NormDist(1,xfc,x);
//   dr = std::max(dr, dr1);
//   dr1 = 0.; // ?????
//   for(size_t i = 0; i < xy_prc.size(); ++i)
//   {
//     dr1 = sqrt(std::pow(xy_prc[i][0] - x[0], 2) + std::pow(xy_prc[i][1] - x[1],2));
//     if(dr1 < rad_prc[i])
//       su = su + 5;
//     dr1 = sqrt(std::pow(xy_prc[i][0] - x[3], 2) + std::pow(xy_prc[i][1] - x[4],2));
//     if(dr1 < rad_prc[i])
//       su = su + 5;
//   }
//   dr1 = sqrt(std::pow(x[0] - x[3], 2) + pow(x[1] - x[4],2));
//   if(dr1 < rad_obc)
//     su = su + 5; 

//   }; 

//   // first step
//   step();


//   while (dr < eps1 || t >= tf)
//   {
//     step();
//   }

//   dr1 = 0;

//   for (size_t i = 0; i < n; ++i)
//     dr1 = dr1 + std::pow(xfc[i] - x[i], 2);

//   return std::sqrt(dr1) + t + su*dt;
  
// }


// double randMToN(double M, double N)
// {
//   return M + (rand()/(RAND_MAX/(N-M)));  
// }

// void PSOAlg()
// {

//   std::cout<<"GAMMA "<<g_<<std::endl;

//   for(size_t j = 0; j < p; ++j)
//   {
//     psoP[0][j] = qq[j];
//     psoV[0][j] = 0.;
//   }

//   std::cout<<1<<std::endl;

//   for(size_t i = 1; i < HH; ++i)
//   {
//     for(size_t j=0; j < p; ++j)
//     {
//       psoP[i][j] = randMToN(-10.0, 10.0) * (qmax[j] - qmin[j]) + qmin[j];
//       psoV[i][j] = 0.; 
//     }
//   }

//   std::cout<<2<<std::endl;

//   for(size_t j = 0; j < HH; ++j)
//   {
//     Fuh[j] = Goal(psoP[j]);
//   }

//   std::cout<<3<<std::endl;

//   float fu0=Fuh[0];
//   for (size_t i = 0; i < PP; ++i)
//   {
//     std::cout<<"3.1 "<<i<<std::endl;
//     for(size_t j = 0; j < HH; ++j)
//     {
//       std::cout<<"3.2 "<<i<<std::endl;
//       // Наилучший из информаторов
//       int infmin = randMToN(0.0, HH);
//       for (int k = 1; k < infs; ++k)
//       {
//         std::cout<<"3.3 "<<i<<std::endl;
//         int kk = randMToN(0.0, HH);
//         if(Fuh[kk] < Fuh[infmin])
//           infmin = kk;
//       }

//       for (size_t k = 0; k < p; ++k)
//       {
//         psoBestInform[k] = psoP[infmin][k];
//       }

//       // best of all
//       //best_inf= psoSolution[0];
//       size_t imin = 0;
//       for(size_t k = 1; k < HH;  ++k)
//       {
//         if (Fuh[k] < Fuh[imin])
//         {
//           imin= k;
//         }
//       }

//       for (size_t k = 0; k < p; ++k)
//       {
//         psoBestGlobal[k] = psoP[imin][k];
//       }

//       // Наилучший в истории
//       if (Fuh[infmin]<fu0)
//       {
//         fu0=Fuh[infmin];
//       }

//       for ( size_t k = 0; k < p; ++k )
//       {
//         psoV[j][k] = alpha * psoV[j][k] + randMToN(-100.0,100.0) * betta * (psoBestInform[k]-psoP[j][k]) + randMToN(-100.0,100.0) * g_ * (psoBestGlobal[k]-psoP[j][k]);
//       }
      
//     }

//     float sigma = 1;
//     for (size_t j = 0; j < HH; ++j)
//     {
//       for(size_t k = 0; k < p; ++k)
//       {
//         qnew[k] = psoP[j][k] + sigma * psoV[j][k];
//         if (qnew[k]>qmax[k]) 
//         {
//           qnew[k]=qmax[k];
//         }
//         else
//         {
//           if (qnew[k]<qmin[k])
//           { 
//             qnew[k]=qmin[k];
//           }
//         }
//       }
//       float Lnew = Goal(qnew);
//       if (Lnew<Fuh[j])
//       {
//         for (size_t k = 0; k < p; k++)
//         {
//           psoP[j][k]=qnew[k];
//         }
//         Fuh[j] = Lnew;
//       }
//     }
//   }

//   std::cout<<4<<std::endl;

//   // Выбор наилучшего решения
//   size_t imin= 0;
//   for (size_t i= 1; i < HH; i++)
//   {
//     if (Fuh[i]<Fuh[imin])
//     {
//       imin= i;
//     }
//   }

//   std::cout<<5<<std::endl;

//   for (size_t i=0; i < p; ++i)
//   {
//     qq[i]=psoP[imin][i];
//   }

//   std::cout<<6<<std::endl;

//   for(auto item : qq)
//   {
//     std::cout<<item<<" ";
//   }
//   std::cout<<std::endl;

// // the end
// }


// int main(int argc, char **argv) 
// {

//   ros::init(argc, argv, "optimal_follower");

//   ros::NodeHandle node;
//   ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 1, model_state_cb);
//   ros::Publisher target_pub = node.advertise<geometry_msgs::PointStamped>("/clicked_point", 1); 


//   // auto path = getPSOpoints(); // Button4Click
//   PSOAlg();

//   ros::Rate rate(1. / 3.);
//   // size_t i = 0;
//   while (ros::ok()) {
//     ros::spinOnce();
//     geometry_msgs::PointStamped PointMsg;
//     std::cout<<"PUBLISH POINT"<<std::endl;
//     target_pub.publish(PointMsg);
//     rate.sleep();
//   }

//   return 0;

// }