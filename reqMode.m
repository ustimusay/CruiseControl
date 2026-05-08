classdef reqMode < Simulink.IntEnumType
    enumeration
        None          (0)
        EnableReq     (1)
        DisableReq    (2)
        ActivateReq   (3)
        DeactivateReq (4)
        ResumeReq     (5)
        SpeedIncReq   (6)
        SpeedDecReq   (7)
    end
    methods (Static)
        function retVal = getDefaultValue()
            retVal = reqMode.None;
        end
        function retVal = getHeaderFile()
            retVal = 'reqMode.h';
        end
        function retVal = addClassNameToEnumNames()
            retVal = false;
        end
    end
end
