/*
 * Copyright (C) 2019 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("freq")) {

    std::cerr << argv[0] << " a PID speed controller." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDLV session ID> "
      << "[--p=<P value>] "
      << "[--d=<D value>] "
      << "[--i=<I value>] "
      << "[--e=<E value, equilibrium point scaling of control value>] "
      << "[--i-limit=<I component limit>] "
      << "[--output-limit-min=<Minimum output value>] "
      << "[--output-limit-max=<Maximum output value>] "
      << "[--input-sender-id=<Sender ID of input message>] "
      << "[--control-sender-id=<Sender ID of control message>] "
      << "[--output-sender-id=<Sender ID of output message>] [--verbose]" 
      << std::endl;
    std::cerr << "Example: " << argv[0] 
      << " --p=1.0 --d=2.0 --cid=111 --freq=50" << std::endl;
    retCode = 1;
  } else {
    uint16_t const cid{static_cast<uint16_t>(
        std::stoi(commandlineArguments["cid"]))};
    uint32_t const freq{static_cast<uint32_t>(
        std::stoi(commandlineArguments["freq"]))};
 
    uint32_t const inputSenderId{
        commandlineArguments.count("input-sender-id") != 0 ?
        static_cast<uint32_t>(
            std::stoi(commandlineArguments["input-sender-id"])) : 0};
    uint32_t const controlSenderId{
        commandlineArguments.count("control-sender-id") != 0 ?
        static_cast<uint32_t>(
            std::stoi(commandlineArguments["control-sender-id"])) : 0};
    uint32_t const outputSenderId{
        commandlineArguments.count("output-sender-id") != 0 ?
        static_cast<uint32_t>(
            std::stoi(commandlineArguments["output-sender-id"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};
    

    bool hasP = commandlineArguments.count("p") != 0;
    bool hasD = commandlineArguments.count("d") != 0;
    bool hasI = commandlineArguments.count("i") != 0;
    bool hasE = commandlineArguments.count("e") != 0;
    bool hasILimit = commandlineArguments.count("i-limit") != 0;
    bool hasOutputLimitMin = 
      commandlineArguments.count("output-limit-min") != 0;
    bool hasOutputLimitMax = 
      commandlineArguments.count("output-limit-max") != 0;

    double p = hasP ? std::stod(commandlineArguments["p"]) : 0.0f;
    double d = hasD ? std::stod(commandlineArguments["d"]) : 0.0f;
    double i = hasI ? std::stod(commandlineArguments["i"]) : 0.0f;
    double e = hasE ? std::stod(commandlineArguments["e"]) : 0.0f;
    double iLimit = hasILimit ? std::stod(
        commandlineArguments["i-limit"]) : 0.0f;
    double outputLimitMin = hasOutputLimitMin ? 
      std::stod(commandlineArguments["output-limit-min"]) : 0.0f;
    double outputLimitMax = hasOutputLimitMax ? 
      std::stod(commandlineArguments["output-limit-max"]) : 0.0f;
    
    double const dt = 1.0 / freq;

    double integral = 0.0;

    double prevError = 0.0;

    bool hasReading{false};
    double reading;
    std::mutex readingMutex;

    bool hasTarget{false};
    double target;
    std::mutex targetMutex;

    cluon::OD4Session od4{cid};

    auto onGroundSpeedReading{
        [&inputSenderId, &reading, &readingMutex, &hasReading, &verbose](
        cluon::data::Envelope &&envelope)
      {
        if (envelope.senderStamp() != inputSenderId) {
          return;
        }
        std::lock_guard<std::mutex> lock(readingMutex);
        auto groundSpeedReading 
          = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(
              std::move(envelope));
        reading = groundSpeedReading.groundSpeed();
        hasReading = true;
        if (verbose) {
          std::cout << "New reading: " << reading << std::endl;
        }
      }};

    auto onGroundSpeedRequest{
        [&controlSenderId, &target, &targetMutex, &hasTarget, &verbose](
        cluon::data::Envelope &&envelope)
      {
        if (envelope.senderStamp() != controlSenderId) {
          return;
        }
        std::lock_guard<std::mutex> lock(targetMutex);
        auto groundSpeedRequest 
          = cluon::extractMessage<opendlv::proxy::GroundSpeedRequest>(
              std::move(envelope));
        target = groundSpeedRequest.groundSpeed();
        hasTarget = true;
        if (verbose) {
          std::cout << "New target set: " << target << std::endl;
        }
      }};

    auto atFrequency{[&outputSenderId, &hasP, &hasD, &hasI, &hasILimit, &hasE,
        &hasOutputLimitMin, &hasOutputLimitMax, &p, &d, &i, &e, &iLimit, 
        &outputLimitMin, &outputLimitMax, &integral, &prevError, &reading, 
        &readingMutex, &hasReading, &target, &targetMutex, &hasTarget,
        &dt, &od4, &verbose]() -> bool
      {
        if (!hasReading || !hasTarget) {
          return true;
        }

        double control = 0.0;

        double error;
        {
          std::lock_guard<std::mutex> lockTarget(targetMutex);
          std::lock_guard<std::mutex> lockReading(readingMutex);
          error = target - reading;
          if (hasE) {
            control += e * target;
          }
        }

        if (hasP) {
          control += p * error;
        }

        if (hasD) {
          control += d * (error - prevError) / dt;
        }

        if (hasI) {
          integral += error * dt;
          if (hasILimit && std::abs(integral) > iLimit) {
            if (integral > 0.0) {
              integral = iLimit;
            } else {
              integral = -iLimit;
            }
          }
          control += i * integral;
        }

        if (hasOutputLimitMin && control < outputLimitMin) {
          control = outputLimitMin;
        }
        
        if (hasOutputLimitMax && control < outputLimitMax) {
          control = outputLimitMax;
        }

        opendlv::proxy::PedalPositionRequest ppr;
        ppr.position(static_cast<float>(control));

        od4.send(ppr, cluon::time::now(), outputSenderId);

        return true;
      }};

    od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),
        onGroundSpeedReading);
    od4.dataTrigger(opendlv::proxy::GroundSpeedRequest::ID(),
        onGroundSpeedRequest);
    od4.timeTrigger(freq, atFrequency);
  }
  return retCode;
}
