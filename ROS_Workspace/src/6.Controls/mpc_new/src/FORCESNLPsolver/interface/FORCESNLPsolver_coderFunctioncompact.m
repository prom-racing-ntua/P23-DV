% FORCESNLPsolver : A fast customized optimization solver.
% 
% Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
% 
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 
% [OUTPUTS] = FORCESNLPsolver(INPUTS) solves an optimization problem where:
% Inputs:
% - xinit - matrix of size [8x1]
% - x0 - matrix of size [300x1]
% - all_parameters - matrix of size [120x1]
% - reinitialize - scalar
% - num_of_threads - scalar
% Outputs:
% - outputs - column vector of length 300
function [outputs] = FORCESNLPsolver(xinit, x0, all_parameters, reinitialize, num_of_threads)
    
    [output, ~, ~] = FORCESNLPsolverBuildable.forcesCall(xinit, x0, all_parameters, reinitialize, num_of_threads);
    outputs = coder.nullcopy(zeros(300,1));
    outputs(1:10) = output.x01;
    outputs(11:20) = output.x02;
    outputs(21:30) = output.x03;
    outputs(31:40) = output.x04;
    outputs(41:50) = output.x05;
    outputs(51:60) = output.x06;
    outputs(61:70) = output.x07;
    outputs(71:80) = output.x08;
    outputs(81:90) = output.x09;
    outputs(91:100) = output.x10;
    outputs(101:110) = output.x11;
    outputs(111:120) = output.x12;
    outputs(121:130) = output.x13;
    outputs(131:140) = output.x14;
    outputs(141:150) = output.x15;
    outputs(151:160) = output.x16;
    outputs(161:170) = output.x17;
    outputs(171:180) = output.x18;
    outputs(181:190) = output.x19;
    outputs(191:200) = output.x20;
    outputs(201:210) = output.x21;
    outputs(211:220) = output.x22;
    outputs(221:230) = output.x23;
    outputs(231:240) = output.x24;
    outputs(241:250) = output.x25;
    outputs(251:260) = output.x26;
    outputs(261:270) = output.x27;
    outputs(271:280) = output.x28;
    outputs(281:290) = output.x29;
    outputs(291:300) = output.x30;
end
