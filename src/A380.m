classdef A380 < handle

%% ClassDescription-------------------------------------------------------%
%A380
%M-File Type               : Class
%Author                    : Stepen
%Version                   : Stepen (stepen.stepen.stepen@gmail.com)
%Date of Creation          : 29 September 2016
%Date of Last Modification : 30 October 2016
%Depedencies               : N/A
%-------------------------------------------------------------------------%
%Description
%A380 creates a handler of patch object for Airbus A380.
% EndOfClassDescription---------------------------------------------------%

%% ClassPropertiesDeclaration---------------------------------------------%
%Declaring variable for coordinate system basis
properties(Constant=true,Access=private,Hidden=true)
    DEF_AXIS_ORIGIN=[0,0,0];
    DEF_AXIS_X=[ 1, 0, 0];
    DEF_AXIS_Y=[ 0,-1, 0];
    DEF_AXIS_Z=[ 0, 0,-1];
end
%Declaring constants for patch generation setup
properties(Constant=true,Access=private,Hidden=true)
    N_VERT4XSEC = 16;
end
%Declaring variables for A380's geometry
properties(Constant=true,Access=private,Hidden=true)
    %Declaring aircraft's center of gravity location
    X_CG = 30;
    Y_CG =  0;
    Z_CG =  0;
    %Declaring parameters for fuselage geometry
    FSLG_XC = [ 0.000;  0.300;  0.900;  1.525;  2.050;  3.075;  3.750;
                4.350;  6.250;  8.750; 11.750; 14.875; 44.750; 50.250;
               54.425; 64.250; 70.500; 71.000; 71.000];
    FSLG_ZC = [ 0.000;  0.000;  0.000;  0.000;  0.000;  0.000;  0.000;
                0.000;  0.000;  0.000;  0.000;  0.000;  0.000;  0.000;
                0.825;  2.750;  4.000;  4.075;  4.075];
    FSLG_RH = [ 0.005;  0.425;  1.025;  1.450;  1.735;  2.175;  2.425;
                2.600;  3.075;  3.425;  3.555;  3.555;  3.555;  3.555;
                3.425;  2.125;  0.750;  0.575;  0.005];
    FSLG_ZU = [ 0.005;  0.600;  1.250;  1.750;  2.000;  3.125;  3.650;
                4.000;  4.725;  5.350;  5.815;  6.000;  6.325;  6.325;
                5.350;  2.750;  1.100;  0.650;  0.005];
    FSLG_ZL = [ 0.005;  0.575;  1.100;  1.450;  1.650;  1.975;  2.125;
                2.225;  2.375;  2.450;  2.450;  2.450;  2.450;  1.825;
                1.850;  1.500;  0.950;  0.650;  0.005];
    %Declaring parameters for wing geometry
    WINGLE_X = [16.2500; 20.8000;  26.5000;  46.5750;  47.5000;  49.0000];
    WINGLE_Y = [ 2.3500;  4.8750;  11.4500;  38.0000;  39.0000;  39.7500];
    WINGLE_Z = [-1.4250; -0.8750;   0.6375;   4.9275;   5.0000;   5.0000];
    WING_C   = [30.2500; 17.8000;  13.7500;   4.6750;   4.1500;   3.0000];
    %Declaring parameters for horizontal stabilizer geometry
    HFINLE_X = [58.6750; 69.5000; 71.5000; 73.4500];
    HFINLE_Y = [ 0.9250; 14.0000; 15.1500; 15.8000];
    HFINLE_Z = [ 3.0000;  3.4500;  3.5250;  3.5500];
    HFIN_C   = [10.1750;  4.7500;  3.2500;  1.6000];
    %Declaring parameters for vertical stabilizer geometry
    VFINLE_X = [47.900; 55.500; 57.350; 68.500; 70.000];
    VFINLE_Y = [ 4.750;  6.875;  7.750; 19.500; 19.875];
    VFIN_C   = [18.850; 12.225; 10.775;  4.975;  3.650];
    %Declaring engine's points of origin
    ENGN_X = [ 31.125;  23.500; 23.500; 31.250];
    ENGN_Y = [-25.750; -14.500; 14.500; 25.750];
    ENGN_Z = [  0.000;  -1.750; -1.750;  0.000];
    %Declaring pylon offset value
    PYLON_DX  = 1.25;
    PYLON_DZ  = 1.55;
    %Declaring parameters for pylon geometry
    PYLONLE_X = [ 0.000; 5.000];
    PYLONLE_Z = [ 0.000; 1.500];
    PYLON_C   = [ 3.500; 7.500];
    %Declaring parameters for engine nacelle geometry
    NCLL_XC = [ 1.350; 0.400; 0.150; 0.075; 0.000; 0.000; 0.175; 0.400;
                1.350; 2.750; 4.450; 5.350; 5.625; 5.625; 5.350; 4.450];
    NCLL_RC = [ 1.300; 1.300; 1.205; 1.205; 1.250; 1.350; 1.515; 1.625;
                1.850; 1.850; 1.615; 1.500; 1.420; 1.350; 1.300; 1.300];
    %Declaring parameters for engine innard geometry
    INND_XC = [ 0.175; 0.235; 0.375; 0.600; 5.075; 6.550; 6.125; 7.700];
    INND_RC = [ 0.005; 0.165; 0.350; 0.450; 1.250; 0.675; 0.575; 0.005];
end
%Declaring variables for A380's airfoil geometry
properties(Constant=true,Access=private,Hidden=true)
    %Declaring wing airfoil geometry
    X_AFL_W = [ 1.0000;  0.8445;  0.6921;  0.5468;  0.4122;  0.4000;
                0.2912;  0.1880;  0.1054;  0.0458;  0.0104;  0.0000;
                0.0141;  0.0520;  0.1124;  0.1939;  0.2944;  0.4000;
                0.4121;  0.5452;  0.6897;  0.8425];
    Z_AFL_W = [ 0.0000;  0.0296;  0.0524;  0.0685;  0.0775;  0.0779;
                0.0785;  0.0712;  0.0575;  0.0396;  0.0199;  0.0000;
               -0.0175; -0.0304; -0.0387; -0.0422; -0.0414; -0.0379;
               -0.0375; -0.0308; -0.0219; -0.0115]*1.2;
    %Declaring stabilizer airfoil geometry
    X_AFL_S = [ 1.0000;  0.8435;  0.6909;  0.5460;  0.4122;  0.2928;
                0.2500;  0.1909;  0.1089;  0.0489;  0.0123;  0.0000;
                0.0123;  0.0489;  0.1089;  0.1909;  0.2500;  0.2928;
                0.4122;  0.5460;  0.6909;  0.8435];
    Z_AFL_S = [ 0.0000;  0.0106;  0.0187;  0.0249;  0.0287;  0.0300;
                0.0297;  0.0284;  0.0241;  0.0176;  0.0094;  0.0000;
               -0.0094; -0.0176; -0.0241; -0.0284; -0.0297; -0.0300;
               -0.0287; -0.0249; -0.0187; -0.0106]*2;
end
%Declaring variables for object properties
properties(SetAccess=private)
    %Patch object handle for A380's polygon
    acfPatchObj;
    %Quiver object handle for A380's body coordinate system
    acfAxisObj;
    %Quiver object handle for A380's earth coordinate system
    earthAxisObj;
    %Lamp object for 3D polygon lighting
    lampObj;
    %A380's CG position in three dimensional space
    cgPos   = A380.DEF_AXIS_ORIGIN;
    %A380's unit vector for positive x axis of body coordinate system
    xAxis   = A380.DEF_AXIS_X;
    %A380's unit vector for positive y axis of body coordinate system
    yAxis   = A380.DEF_AXIS_Y;
    %A380's unit vector for positive z axis of body coordinate system
    zAxis   = A380.DEF_AXIS_Z;
    %A380's template x axis coordinate data for polygon vertexes
    xData_0 = zeros(0,1);
    %A380's current x axis coordinate data for polygon vertexes
    xData_c = zeros(0,1);
    %A380's template y axis coordinate data for polygon vertexes
    yData_0 = zeros(0,1);
    %A380's current y axis coordinate data for polygon vertexes
    yData_c = zeros(0,1);
    %A380's template z axis coordinate data for polygon vertexes
    zData_0 = zeros(0,1);
    %A380's current y axis coordinate data for polygon vertexes
    zData_c = zeros(0,1);
    %A380's face connectivity data for polygon faces
    fData   = zeros(0,1);
    %A380's vertex color data for polygon faces
    cData   = zeros(0,1);
end
%Declaring variables for A380's general dimension
properties(Dependent=true)
    %A380's overall fuselage length
    overallLength;
    %A380's wingspan
    wingSpan;
end
% EndOfClassPropertiesDeclaration-----------------------------------------%

%% ClassConstructorDeclaration--------------------------------------------%
methods(Access=public)
    function this=A380(varargin)
    % Constructs new instances of A380 patch handler object.
        %Checking input argument
        switch nargin
            case 0
                
            case 3
                
            otherwise
                error('Unexpected number of input arguments!');
        end
        %Preallocating temporary variable to store patch data
        ac_x=zeros(0,1);
        ac_y=zeros(0,1);
        ac_z=zeros(0,1);
        ac_f=zeros(0,1);
        ac_c=zeros(0,1);
        %Generating vertex data for fuselage's complete geometry
        [x,y,z,f]=A380.GenerateBodyPatch(A380.FSLG_XC,A380.FSLG_ZC,...
                                         A380.FSLG_RH,...
                                         A380.FSLG_ZU,A380.FSLG_ZL);
        %Adding fuselage geometry into patch data
        ac_x=[ac_x;x];
        ac_y=[ac_y;y];
        ac_z=[ac_z;z];
        ac_f=[ac_f;f];
        ac_c=[ac_c;[ones(numel(x),1),ones(numel(x),1),ones(numel(x),1)]];
        %Generating vertex data for wing's complete geometry
        [x,y,z,f]=A380.GenerateSurfacePatch(A380.X_AFL_W,A380.Z_AFL_W,...
                                            A380.WINGLE_X,...
                                            A380.WINGLE_Y,...
                                            A380.WINGLE_Z,...
                                            A380.WING_C,true);
        %Adding wing geometry into patch data
        last=numel(ac_x);
        ac_x=[ac_x;x];
        ac_y=[ac_y;y];
        ac_z=[ac_z;z];
        ac_f=[ac_f;f+last];
        ac_c=[ac_c;[0.8*ones(numel(x),1),...
                    0.8*ones(numel(x),1),...
                    0.8*ones(numel(x),1)]];
        %Generating vertex data for horizontal tail's complete geometry
        [x,y,z,f]=A380.GenerateSurfacePatch(A380.X_AFL_S,A380.Z_AFL_S,...
                                            A380.HFINLE_X,...
                                            A380.HFINLE_Y,...
                                            A380.HFINLE_Z,...
                                            A380.HFIN_C,true);
        %Adding horizontal tail geometry into patch data
        last=numel(ac_x);
        ac_x=[ac_x;x];
        ac_y=[ac_y;y];
        ac_z=[ac_z;z];
        ac_f=[ac_f;f+last];
        ac_c=[ac_c;[0.8*ones(numel(x),1),...
                    0.8*ones(numel(x),1),...
                    0.8*ones(numel(x),1)]];
        %Generating vertex data for vertical tail's complete geometry
        [x,z,y,f]=A380.GenerateSurfacePatch(A380.X_AFL_S,A380.Z_AFL_S,...
                                            A380.VFINLE_X,...
                                            A380.VFINLE_Y,...
                                            zeros(size(A380.VFINLE_X)),...
                                            A380.VFIN_C,false);
        %Adding vertical tail geometry into patch data
        last=numel(ac_x);
        ac_x=[ac_x;x];
        ac_y=[ac_y;y];
        ac_z=[ac_z;z];
        ac_f=[ac_f;f+last];
        ac_c=[ac_c;[zeros(numel(x),1),zeros(numel(x),1),ones(numel(x),1)]];
        %Generating vertex data for engine pylon's complete geometry
        [x,z,y,f]=A380.GenerateSurfacePatch(A380.X_AFL_S,A380.Z_AFL_S,...
                                            A380.PYLONLE_X,...
                                            A380.PYLONLE_Z,...
                                            zeros(size(A380.PYLONLE_X)),...
                                            A380.PYLON_C,false);
        %Adding engine pylon into patch data
        for id_engine=1:numel(A380.ENGN_X)
            last = numel(ac_x);
            ac_x = [ac_x;x+A380.ENGN_X(id_engine)+A380.PYLON_DX];
            ac_y = [ac_y;y+A380.ENGN_Y(id_engine)];
            ac_z = [ac_z;z+A380.ENGN_Z(id_engine)+A380.PYLON_DZ];
            ac_f = [ac_f;f+last];
            ac_c = [ac_c;[ones(numel(x),1),...
                                      ones(numel(x),1),...
                                      ones(numel(x),1)]];
        end
        %Generating vertex data for engine nacelle's complete geometry
        [x,y,z,f]=A380.GenerateBodyPatch(A380.NCLL_XC,...
                                         zeros(size(A380.NCLL_XC)),...
                                         A380.NCLL_RC,...
                                         A380.NCLL_RC,A380.NCLL_RC);
        %Adding engine nacelle into patch data
        for id_engine=1:numel(A380.ENGN_X)
            last = numel(ac_x);
            ac_x = [ac_x;x+A380.ENGN_X(id_engine)];
            ac_y = [ac_y;y+A380.ENGN_Y(id_engine)];
            ac_z = [ac_z;z+A380.ENGN_Z(id_engine)];
            ac_f = [ac_f;f+last];
            ac_c = [ac_c;[ones(numel(x),1),...
                                      ones(numel(x),1),...
                                      ones(numel(x),1)]];
        end
        %Generating vertex data for engine innard's complete geometry
        [x,y,z,f]=A380.GenerateBodyPatch(A380.INND_XC,...
                                         zeros(size(A380.INND_XC)),...
                                         A380.INND_RC,...
                                         A380.INND_RC,A380.INND_RC);
        %Adding engine innard into patch data
        for id_engine=1:numel(A380.ENGN_X)
            last = numel(ac_x);
            ac_x = [ac_x;x+A380.ENGN_X(id_engine)];
            ac_y = [ac_y;y+A380.ENGN_Y(id_engine)];
            ac_z = [ac_z;z+A380.ENGN_Z(id_engine)];
            ac_f = [ac_f;f+last];
            ac_c = [ac_c;[0.5*ones(numel(x),1),...
                                      0.5*ones(numel(x),1),...
                                      0.5*ones(numel(x),1)]];
        end
        %Centering vertex data at the aircraft's center of gravity
        ac_x=ac_x-A380.X_CG;
        ac_y=ac_y-A380.Y_CG;
        ac_z=ac_z-A380.Z_CG;
        %Reversing aircraft's orientation to match body system coordinate
        ac_x=-ac_x;
        %Storing original vertex data as object properties
        this.xData_0=ac_x;
        this.xData_c=ac_x;
        this.yData_0=ac_y;
        this.yData_c=ac_y;
        this.zData_0=ac_z;
        this.zData_c=ac_z;
        this.fData=ac_f;
        this.cData=ac_c;
        %Generating patch object
        this.DrawAircraft();
    end
end
% EndOfClassConstructorDeclaration----------------------------------------%

%% PublicMethodDeclaration------------------------------------------------%
methods(Access=public)
    function ResetAircraft(this)
    % Resets A380's position and attitude.
    % ResetAircraft() resets the current A380 patch's object position to
    % (0,0,0) and its rotation angle to 0.
        this.cgPos=A380.DEF_AXIS_ORIGIN;
        this.xAxis=A380.DEF_AXIS_X;
        this.yAxis=A380.DEF_AXIS_Y;
        this.zAxis=A380.DEF_AXIS_Z;
        this.xData_c=this.xData_0;
        this.yData_c=this.yData_0;
        this.zData_c=this.zData_0;
        this.DrawAircraft;
    end
    function MoveForward(this,distance)
    % Moves A380's center of gravity and the entire aircraft forward.
    % MoveForward(dist) moves the aircraft forward in the x axis direction
    % of body coordinate system by given distance value.
        newpos=this.cgPos+(this.xAxis*distance);
        this.MoveTo(newpos(1),newpos(2),newpos(3));
    end
    function MoveTo(this,x,y,z)
    % Moves A380's center of gravity and the entire aircraft.
    % MoveTo(x,y,z) moves the aircraft to given location x,y,z.
        offset=[x,y,z]-this.cgPos;
        this.cgPos=[x,y,z];
        this.xData_c=this.xData_c+offset(1);
        this.yData_c=this.yData_c+offset(2);
        this.zData_c=this.zData_c+offset(3);
        this.DrawAircraft;
    end
    function AddRoll(this,angle)
    % Rotates A380 around its x-body axis.
    % Rotate(angle) rotates the aircraft around its body X-axis by given
    % angle value.
        %Rotating aircraft vertex data
        [x,y,z]=A380.Rotate(this.xData_c-this.cgPos(1),...
                            this.yData_c-this.cgPos(2),...
                            this.zData_c-this.cgPos(3),...
                            this.xAxis,angle);
        this.xData_c=x+this.cgPos(1);
        this.yData_c=y+this.cgPos(2);
        this.zData_c=z+this.cgPos(3);
        %Rotating body axis
        [x,y,z]=A380.Rotate(this.yAxis(1),this.yAxis(2),this.yAxis(3),...
                            this.xAxis,angle);
        this.yAxis=[x,y,z];
        [x,y,z]=A380.Rotate(this.zAxis(1),this.zAxis(2),this.zAxis(3),...
                            this.xAxis,angle);
        this.zAxis=[x,y,z];
        %Updating aircraft patch object
        this.DrawAircraft;
    end
    function AddPitch(this,angle)
    % Rotates A380 around its y-body axis.
    % Rotate(angle) rotates the aircraft around its body Y-axis by given
    % angle value.
        %Rotating aircraft vertex data
        [x,y,z]=A380.Rotate(this.xData_c-this.cgPos(1),...
                            this.yData_c-this.cgPos(2),...
                            this.zData_c-this.cgPos(3),...
                            this.yAxis,angle);
        this.xData_c=x+this.cgPos(1);
        this.yData_c=y+this.cgPos(2);
        this.zData_c=z+this.cgPos(3);
        %Rotating body axis
        [x,y,z]=A380.Rotate(this.xAxis(1),this.xAxis(2),this.xAxis(3),...
                            this.yAxis,angle);
        this.xAxis=[x,y,z];
        [x,y,z]=A380.Rotate(this.zAxis(1),this.zAxis(2),this.zAxis(3),...
                            this.yAxis,angle);
        this.zAxis=[x,y,z];
        %Updating aircraft patch object
        this.DrawAircraft;
    end
    function AddYaw(this,angle)
    % Rotates A380 around its z-body axis.
    % Rotate(angle) rotates the aircraft around its body Y-axis by given
    % angle value.
        %Rotating aircraft vertex data
        [x,y,z]=A380.Rotate(this.xData_c-this.cgPos(1),...
                            this.yData_c-this.cgPos(2),...
                            this.zData_c-this.cgPos(3),...
                            this.zAxis,angle);
        this.xData_c=x+this.cgPos(1);
        this.yData_c=y+this.cgPos(2);
        this.zData_c=z+this.cgPos(3);
        %Rotating body axis
        [x,y,z]=A380.Rotate(this.xAxis(1),this.xAxis(2),this.xAxis(3),...
                            this.zAxis,angle);
        this.xAxis=[x,y,z];
        [x,y,z]=A380.Rotate(this.yAxis(1),this.yAxis(2),this.yAxis(3),...
                            this.zAxis,angle);
        this.yAxis=[x,y,z];
        %Updating aircraft patch object
        this.DrawAircraft;
    end
    function DrawAircraft(this)
    % Draw graphical patch object and re-create deleted patch object.
    % DrawAircraft() update the patch object of A380 or recreate one if
    % it does not already exist.
        if (~isempty(this.acfPatchObj))
        if (isvalid(this.acfPatchObj))
            set(this.acfPatchObj,'XData',this.xData_c);
            set(this.acfPatchObj,'YData',this.yData_c);
            set(this.acfPatchObj,'ZData',this.zData_c);
            set(this.acfPatchObj,'Faces',this.fData);
            axis('equal',[this.cgPos(1)-0.75*this.overallLength,...
                          this.cgPos(1)+0.75*this.overallLength,...
                          this.cgPos(2)-0.75*this.overallLength,...
                          this.cgPos(2)+0.75*this.overallLength,...
                          this.cgPos(3)-0.75*this.overallLength,...
                          this.cgPos(3)+0.75*this.overallLength]);
            grid on;
            this.DrawAxis();
            return;
        end
        end
        this.acfPatchObj=patch('XData',this.xData_c,...
                               'YData',this.yData_c,...
                               'ZData',this.zData_c,...
                               'Faces',this.fData,...
                               'FaceColor','flat',...
                               'FaceVertexCData',this.cData,...
                               'FaceLighting','gouraud',...
                               'LineStyle','none');
        this.lampObj=camlight('headlight');
        axis('equal',[this.cgPos(1)-this.wingSpan,...
                      this.cgPos(1)+this.wingSpan,...
                      this.cgPos(2)-this.wingSpan,...
                      this.cgPos(2)+this.wingSpan,...
                      this.cgPos(3)-this.wingSpan,...
                      this.cgPos(3)+this.wingSpan]);
        grid on;
        this.DrawAxis();
    end
    function DrawAxis(this)
    % Draw quiver object to display A380's coordinate system.
    % DrawAxis() draws body and earth coordinate axis for A380.
        if (~isempty(this.acfAxisObj))
        if (isvalid(this.acfAxisObj))
            set(this.earthAxisObj,'XData',this.cgPos(1)*ones(3,1));
            set(this.earthAxisObj,'YData',this.cgPos(2)*ones(3,1));
            set(this.earthAxisObj,'ZData',this.cgPos(3)*ones(3,1));
            set(this.earthAxisObj,...
                'UData',0.5*this.wingSpan*[A380.DEF_AXIS_X(1);...
                                           A380.DEF_AXIS_Y(1);...
                                           A380.DEF_AXIS_Z(1)]);
            set(this.earthAxisObj,...
                'VData',0.5*this.wingSpan*[A380.DEF_AXIS_X(2);...
                                           A380.DEF_AXIS_Y(2);...
                                           A380.DEF_AXIS_Z(2)]);
            set(this.earthAxisObj,...
                'WData',0.5*this.wingSpan*[A380.DEF_AXIS_X(3);...
                                           A380.DEF_AXIS_Y(3);...
                                           A380.DEF_AXIS_Z(3)]);
            set(this.acfAxisObj,'XData',this.cgPos(1)*ones(3,1));
            set(this.acfAxisObj,'YData',this.cgPos(2)*ones(3,1));
            set(this.acfAxisObj,'ZData',this.cgPos(3)*ones(3,1));
            set(this.acfAxisObj,...
                'UData',0.5*this.wingSpan*[this.xAxis(1);...
                                           this.yAxis(1);...
                                           this.zAxis(1)]);
            set(this.acfAxisObj,...
                'VData',0.5*this.wingSpan*[this.xAxis(2);...
                                           this.yAxis(2);...
                                           this.zAxis(2)]);
            set(this.acfAxisObj,...
                'WData',0.5*this.wingSpan*[this.xAxis(3);...
                                           this.yAxis(3);...
                                           this.zAxis(3)]);
            return;
        end
        end
        hold on;
        this.earthAxisObj=quiver3(this.cgPos(1)*ones(3,1),...
                                  this.cgPos(2)*ones(3,1),...
                                  this.cgPos(3)*ones(3,1),...
                                  0.5*this.wingSpan*[this.DEF_AXIS_X(1);...
                                                     this.DEF_AXIS_Y(1);...
                                                     this.DEF_AXIS_Z(1)],...
                                  0.5*this.wingSpan*[this.DEF_AXIS_X(2);...
                                                     this.DEF_AXIS_Y(2);...
                                                     this.DEF_AXIS_Z(2)],...
                                  0.5*this.wingSpan*[this.DEF_AXIS_X(3);...
                                                     this.DEF_AXIS_Y(3);...
                                                     this.DEF_AXIS_Z(3)],...
                                  'k');
        this.acfAxisObj=quiver3(this.cgPos(1)*ones(3,1),...
                                this.cgPos(2)*ones(3,1),...
                                this.cgPos(3)*ones(3,1),...
                                0.5*this.wingSpan*[this.xAxis(1);...
                                                   this.yAxis(1);...
                                                   this.zAxis(1)],...
                                0.5*this.wingSpan*[this.xAxis(2);...
                                                   this.yAxis(2);...
                                                   this.zAxis(2)],...
                                0.5*this.wingSpan*[this.xAxis(3);...
                                                   this.yAxis(3);...
                                                   this.zAxis(3)],...
                                'b');
                            xlabel('x');
                            ylabel('y');
                            zlabel('z');
        hold off;
    end
    function ShowBirdEyeView(this)
    % Updates plot to show the birdeye view of A380.
    % ShowPortSide() create patch object of A380 if it have not already
    % existed and shows its birdeye view.
        %Drawing aircraft patch object
        this.DrawAircraft();
        %Get parent axes handle
        axesHandle=get(this.acfPatchObj,'Parent');
        %Set axes camera manually
        set(axesHandle,'CameraPosition',...
                       this.cgPos-(this.DEF_AXIS_Z*this.wingSpan));
        set(axesHandle,'CameraTarget',[this.cgPos]);
        camlight(this.lampObj,'headlight');
    end
    function ShowTopSide(this)
    % Updates plot to show the top side of A380.
    % ShowPortSide() create patch object of A380 if it have not already
    % existed and shows its top side view.
        %Drawing aircraft patch object
        this.DrawAircraft();
        %Get parent axes handle
        axesHandle=get(this.acfPatchObj,'Parent');
        %Set axes camera manually
        set(axesHandle,'CameraPosition',...
                       this.cgPos-(this.zAxis*this.wingSpan));
        set(axesHandle,'CameraTarget',[this.cgPos]);
        camlight(this.lampObj,'headlight');
    end
    function ShowNoseSide(this)
    % Updates plot to show the nose side of A380.
    % ShowPortSide() create patch object of A380 if it have not already
    % existed and shows its nose side view.
        %Drawing aircraft patch object
        this.DrawAircraft();
        %Get parent axes handle
        axesHandle=get(this.acfPatchObj,'Parent');
        %Set axes camera manually
        set(axesHandle,'CameraPosition',...
                       this.cgPos+(this.xAxis*this.wingSpan));
        set(axesHandle,'CameraTarget',[this.cgPos]);
        camlight(this.lampObj,'headlight');
    end
    function ShowRearView(this)
    % Updates plot to show the rear view of A380
    % ShowTailSide() create patch object of A380 if it have not already
    % existed and shows its rear view.
        %Drawing aircraft patch object
        this.DrawAircraft();
        %Get parent axes handle
        axesHandle=get(this.acfPatchObj,'Parent');
        %Set axes camera manually
        camAxis=this.xAxis;
        camAxis(3)=0;
        set(axesHandle,'CameraPosition',...
                       this.cgPos-(camAxis*this.wingSpan));
        set(axesHandle,'CameraTarget',[this.cgPos]);
        camlight(this.lampObj,'headlight');
    end
    function ShowTailSide(this)
    % Updates plot to show the tail side of A380
    % ShowTailSide() create patch object of A380 if it have not already
    % existed and shows its tail side view.
        %Drawing aircraft patch object
        this.DrawAircraft();
        %Get parent axes handle
        axesHandle=get(this.acfPatchObj,'Parent');
        %Set axes camera manually
        set(axesHandle,'CameraPosition',...
                       this.cgPos-(this.xAxis*this.wingSpan));
        set(axesHandle,'CameraTarget',[this.cgPos]);
        camlight(this.lampObj,'headlight');
    end
    function ShowPortSide(this)
    % Updates plot to show the port side of A380.
    % ShowPortSide() create patch object of A380 if it have not already
    % existed and shows its port side view.
        %Drawing aircraft patch object
        this.DrawAircraft();
        %Get parent axes handle
        axesHandle=get(this.acfPatchObj,'Parent');
        %Set axes camera manually
        set(axesHandle,'CameraPosition',...
                       this.cgPos-(this.yAxis*this.wingSpan));
        set(axesHandle,'CameraTarget',[this.cgPos]);
        camlight(this.lampObj,'headlight');
    end
    function ShowStarboardSide(this)
    % Updates plot to show the starboard side of A380.
    % ShowStarboardSide() create patch object of A380 if it have not
    % already existed and shows its starboard side view.
        %Drawing aircraft patch object
        this.DrawAircraft();
        %Get parent axes handle
        axesHandle=get(this.acfPatchObj,'Parent');
        %Set axes camera manually
        set(axesHandle,'CameraPosition',...
                       this.cgPos+(this.yAxis*this.wingSpan));
        set(axesHandle,'CameraTarget',[this.cgPos]);
        camlight(this.lampObj,'headlight');
    end
    
    % My show function
    function MyShow(this)
        %Drawing aircraft patch object
        this.DrawAircraft();
        %set Azimuth and Elevation of the point of view
        view([40 10])
    end
end
% EndOfPublicMethodDeclaration--------------------------------------------%

%% Setter&GetterMethodDeclaration-----------------------------------------%
methods
    function value=get.overallLength(this)
    % Declares getter function for A380's overall length.
        value=max(this.FSLG_XC);
    end
    function value=get.wingSpan(this)
    % Declares getter function for A380's wing span.
        value=2*max(this.WINGLE_Y);
    end
end
% EndOfSetter&GetterMethodDeclaration-------------------------------------%

%% StaticMethodDeclaration------------------------------------------------%
methods(Static=true,Hidden=true)
    function [x,y,z,face]=GenerateBodyPatch(xc,zc,rh,zu,zl)
    % Generates body patch vertexes and faces from given body parameters.
        %Declaring default value for cross-section generation
        theta = 0:(2*pi/A380.N_VERT4XSEC):((2*pi)-(2*pi/A380.N_VERT4XSEC));
        %Preallocating array
        n_xsec=numel(xc);
        n_vert=numel(theta);
        x = zeros((n_xsec*n_vert),1);
        y = zeros((n_xsec*n_vert),1);
        z = zeros((n_xsec*n_vert),1);
        for id_xsec=1:n_xsec
            %Calculating vertexes coordinates for each body cross-section
            x_temp            = ones(size(theta))*xc(id_xsec);
            y_temp            = rh(id_xsec)*cos(theta);
            z_temp            = zeros(size(theta));
            z_temp(theta<=pi) = zc(id_xsec)+...
                                (zu(id_xsec)*sin(theta(theta<=pi)));
            z_temp(theta>pi)  = zc(id_xsec)+...
                                (zl(id_xsec)*sin(theta(theta>pi)));
            %Inserting cross-section vertexes to coordinate table
            x(((id_xsec-1)*n_vert)+1:(id_xsec*n_vert)) = x_temp;
            y(((id_xsec-1)*n_vert)+1:(id_xsec*n_vert)) = y_temp;
            z(((id_xsec-1)*n_vert)+1:(id_xsec*n_vert)) = z_temp;
        end
        %Generating body face connectivity data
        face = zeros((n_xsec-1)*n_vert,4);
        for id_xsec=1:n_xsec-1
            order     = ((((id_xsec-1)*n_vert+1)):(id_xsec*n_vert))';
            order_off = [order(2:end);order(1)];
            mark1     = ((id_xsec-1)*n_vert)+1;
            mark2     = (id_xsec*n_vert);
            face(mark1:mark2,1) = order;
            face(mark1:mark2,2) = order_off;
            face(mark1:mark2,3) = order_off+n_vert;
            face(mark1:mark2,4) = order+n_vert;
        end
    end
    function [x,y,z,face]=GenerateSurfacePatch(x_afl,z_afl,...
                                               x_LE,y_LE,z_LE,chord,sym)
    % Generates wing patch vertexes and faces from given wing parameters.
        %Adding zero chord cross-section to close tip surface
        x_LE  = [x_LE; x_LE(end)+(0.5*chord(end))];
        y_LE  = [y_LE; y_LE(end)];
        z_LE  = [z_LE; z_LE(end)];
        chord = [chord; 0];
        %Mirroring cross-section data for symmetric lifting surface
        if (sym)
            x_LE  = [x_LE(end:-1:1);x_LE];
            y_LE  = [-y_LE(end:-1:1);y_LE];
            z_LE  = [z_LE(end:-1:1);z_LE];
            chord = [chord(end:-1:1);chord];
        end
        %Generating lifting surface vertex data from given geometry data
        n_xsec  = numel(x_LE);
        n_vert  = numel(x_afl);
        x  = zeros(2*n_xsec*n_vert,1);
        y  = zeros(2*n_xsec*n_vert,1);
        z  = zeros(2*n_xsec*n_vert,1);
        counter = 0;
        for id_xsec=1:n_xsec
            counter = counter+1;
            x(((counter-1)*n_vert)+1:counter*n_vert)=...
                x_LE(id_xsec)+(x_afl*chord(id_xsec));
            y(((counter-1)*n_vert)+1:counter*n_vert)=...
                ones(size(x_afl))*y_LE(id_xsec);
            z(((counter-1)*n_vert)+1:counter*n_vert)=...
                z_LE(id_xsec)+z_afl*chord(id_xsec);
        end
        %Generating lifting surface face connectivity data
        face = zeros((n_xsec-1)*n_vert,4);
        for id_xsec=1:n_xsec-1
            order     = ((id_xsec-1)*n_vert)+(1:n_vert)';
            order_off = ((id_xsec-1)*n_vert)+[2:n_vert,1]';
            mark1     = ((id_xsec-1)*n_vert)+1;
            mark2     = (id_xsec*n_vert);
            face(mark1:mark2,1) = order;
            face(mark1:mark2,2) = order_off;
            face(mark1:mark2,3) = order_off+n_vert;
            face(mark1:mark2,4) = order+n_vert;
        end
    end
    function [x_out,y_out,z_out]=Rotate(x_in,y_in,z_in,rot_ax,rot_angle)
    % Performs Rodriguez rotation for given x,y,z location by using given
    % rotation axis and rotation angle.
        %Normalizing rotation axis
        rot_ax=rot_ax/sum(rot_ax.^2);
        %Calculating cross product between vertexes and rotation axis
        crossprod=[(rot_ax(2)*z_in)-(rot_ax(3)*y_in),...
                   (rot_ax(3)*x_in)-(rot_ax(1)*z_in),...
                   (rot_ax(1)*y_in)-(rot_ax(2)*x_in)];
        %Calculating dot product between vertexes and rotation axis
        dotprod=(rot_ax(1)*x_in)+(rot_ax(2)*y_in)+(rot_ax(3)*z_in);
        axdotprod=[rot_ax(1)*dotprod,rot_ax(2)*dotprod,rot_ax(3)*dotprod];
        %Calculating rotation result
        xyz=[x_in,y_in,z_in];
        xyz_rot=(xyz*cos(rot_angle))+...
                (crossprod*sin(rot_angle))+...
                (axdotprod*(1-cos(rot_angle)));
        x_out=xyz_rot(:,1);
        y_out=xyz_rot(:,2);
        z_out=xyz_rot(:,3);
    end
end
% EndOfStaticMethodDeclaration--------------------------------------------%

end