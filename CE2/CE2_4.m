clc; clear variables; close all;


javaaddpath('C:\Program Files\Mosek/9.2/tools/platform/win64x86/bin/mosek.jar')
%Mirkos Path, @Silvio: comment mine wes muesch ändere vorem pushe
addpath 'C:\Program Files\MATLAB\R2018a\Mosek\9.2\toolbox\R2015a'
%@Silvio: Muesch dr ganz Mosek-Folder wo i Program Files hesch ou i di MATLAB
%folder inekopiere

import mosek.fusion.*;
M = Model()