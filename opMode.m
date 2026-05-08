classdef opMode < Simulink.IntEnumType
    enumeration
        Disabled  (0)
        Enabled   (1)
        Activated (2)
    end
    methods (Static)
        function retVal = getDefaultValue()
            retVal = opMode.Disabled;
        end
        function retVal = getHeaderFile()
            retVal = 'opMode.h';
        end
        function retVal = addClassNameToEnumNames()
            retVal = false;
        end
    end
end
