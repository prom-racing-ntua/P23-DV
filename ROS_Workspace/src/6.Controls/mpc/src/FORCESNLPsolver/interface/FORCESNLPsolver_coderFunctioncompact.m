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
% - xinit - matrix of size [9x1]
% - x0 - matrix of size [360x1]
% - all_parameters - matrix of size [120x1]
% - reinitialize - scalar
% - num_of_threads - scalar
% Outputs:
% - outputs - column vector of length 360
function [outputs] = FORCESNLPsolver(xinit, x0, all_parameters, reinitialize, num_of_threads)
    
    [output, ~, ~] = FORCESNLPsolverBuildable.forcesCall(xinit, x0, all_parameters, reinitialize, num_of_threads);
    outputs = coder.nullcopy(zeros(360,1));
    outputs(1:12) = output.x01;
    outputs(13:24) = output.x02;
    outputs(25:36) = output.x03;
    outputs(37:48) = output.x04;
    outputs(49:60) = output.x05;
    outputs(61:72) = output.x06;
    outputs(73:84) = output.x07;
    outputs(85:96) = output.x08;
    outputs(97:108) = output.x09;
    outputs(109:120) = output.x10;
    outputs(121:132) = output.x11;
    outputs(133:144) = output.x12;
    outputs(145:156) = output.x13;
    outputs(157:168) = output.x14;
    outputs(169:180) = output.x15;
    outputs(181:192) = output.x16;
    outputs(193:204) = output.x17;
    outputs(205:216) = output.x18;
    outputs(217:228) = output.x19;
    outputs(229:240) = output.x20;
    outputs(241:252) = output.x21;
    outputs(253:264) = output.x22;
    outputs(265:276) = output.x23;
    outputs(277:288) = output.x24;
    outputs(289:300) = output.x25;
    outputs(301:312) = output.x26;
    outputs(313:324) = output.x27;
    outputs(325:336) = output.x28;
    outputs(337:348) = output.x29;
    outputs(349:360) = output.x30;
end
