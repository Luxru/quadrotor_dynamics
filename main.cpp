#include <flightlib/objects/quadrotor.hpp>
#include <flightlib/common/command.hpp>
#include <flightlib/common/quad_state.hpp>
#include <chrono>
#include <iostream>

int main() {
    flightlib::Quadrotor quadrotor(
        "/Users/yanxr/Desktop/mocap/src/quadrotor/configs/quadrotor.yaml"
    );
    //run sim for 10s
    const auto start = std::chrono::high_resolution_clock::now();
    int count = 0;
    double sim_dt = 0.01;
    //turnoff sync with stdio
    // std::ios::sync_with_stdio(false);
    //hover to 1m
    flightlib::QuadState state;
    quadrotor.getState(&state);
    state.p = Eigen::Vector3f(0,0,5);
    quadrotor.setState(state);
    while (count < 300.0) {
        quadrotor.getState(&state);
        flightlib::Command cmd( sim_dt*count,flightlib::CommandType::RATES_THRUST);
        cmd.collective_thrust = 0;
        cmd.omega = Eigen::Vector3f(1,1,1);//under actuated

        //and calulate time cost of simulation
        const auto sim_start = std::chrono::high_resolution_clock::now();
        quadrotor.run(cmd,sim_dt);
        const auto sim_end = std::chrono::high_resolution_clock::now();
        std::cout<<"sim time cost: "<<std::chrono::duration_cast<std::chrono::microseconds>(sim_end - sim_start).count()<<"us"<<std::endl;
        std::cout<<quadrotor<<std::endl;
        count++;
        
        if((state.w-cmd.omega).norm() < 1e-3){
            std::cout<<"sim time elapsed: "<<count*sim_dt<<"s"<<std::endl;
            break;
        }
    }
    std::cout<<"total sim count: "<<count<<", sim time elapsed: "<<count*sim_dt<<"s"<<std::endl;
    return 0;
}