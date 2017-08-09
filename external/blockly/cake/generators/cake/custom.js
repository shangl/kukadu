'use strict';

goog.provide('Blockly.cake.custom');
goog.require('Blockly.cake');

var skillCounter = 0;

Blockly.cake['objectposition'] = function (block) {
    var variableName = block.getFieldValue("VariableName");
    var tmpVariableName = variableName + "tmp";
    var objectName = block.getFieldValue("ObjectType");

    return "auto " + variableName + " = kukadu::PoseEstimatorFactory::get().getPoseFor(\"" + objectName + "\").pose;\n";
};


Blockly.cake['skillloader'] = function (block) {
    Blockly.cake.definitions_['include_cake_string'] =
        '#include <stdlib.h>\n#include <kukadu/kukadu.hpp>';

    var hardwareSelection = Blockly.cake.valueToCode(block, 'HARDWARE', Blockly.cake.ORDER_ATOMIC);
    var splitHardware = hardwareSelection.split(", ");

    for(var i = 0; i < splitHardware.length; i++) {
        Blockly.cake.neededHardware_[splitHardware[i]] = "auto " + splitHardware[i] + " = hardwareFactory.loadHardware(\"" + splitHardware[i] + "\");\n";
    }

    var hardwareCode = "";
    var hardwareVariableNames = [];

    for (var i = 0; i < splitHardware.length; i++) {
        var hardwareVariableName = "sLeftQueue" + skillCounter + i;
        hardwareCode += "auto " + hardwareVariableName + " = hardwareFactory.loadHardware(\"" + splitHardware[i] + "\");\n" +
            hardwareVariableName + "->install();\n" +
            hardwareVariableName + "->start();\n";
        hardwareVariableNames.push(hardwareVariableName);
    }

    var skillCode = "";
    var skillSelection = block.getCurrentSkill();
    if (typeof skillSelection !== 'undefined') {
        skillCode = "\nauto skill" + skillCounter + " = kukadu::SkillFactory::get().loadSkill(\"" + skillSelection.name + "\", {";
        for (var j = 0; j < hardwareVariableNames.length - 1; j++) {
            skillCode += hardwareVariableNames[j] + ", ";
        }

        skillCode += hardwareVariableNames[hardwareVariableNames.length - 1] + "});\n\n";

        var skillAttributeInformation = skillSelection.getAttributes();

        var additionalLimitForVectors = 0;
        var attributes = getValuesFromMap(skillAttributeInformation);

        for (var i = 0; i < attributes.length + additionalLimitForVectors; i++) {
            var attribute = attributes[i];
            var attributeCode = "";
            var everyAttributeIsSet = true;
            if (attribute.dataType === "std::vector< double >" || attribute.dataType === "std::vector< int >" || attribute.dataType === "std::vector< string >") {
                var isString = attribute.dataType === "std::vector< string >";
                var vectorsize = 0;
                var roboConfig = Databaseloader.roboConfigMap[idsToKey(block._hardwareIds)];
                roboConfig.hardwareInOrder.forEach(function (hardware) {
                    vectorsize = hardware.degOfFreedom > vectorsize ? hardware.degOfFreedom : vectorsize;
                });

                attributeCode = "{";

                for (var j = 0; j < vectorsize - 1; j++) {
                    attributeCode += isString ? "'" : "";
                    var attributeValue = block.getFieldValue('attribute' + i++);
                    additionalLimitForVectors++;

                    if (attributeValue === "not defined") {
                        everyAttributeIsSet = false;
                    }

                    attributeCode += attributeValue + ", ";
                    attributeCode += isString ? "'" : "";
                }

                var attributeValue = block.getFieldValue('attribute' + i);
                everyAttributeIsSet = attributeValue !== "not defined";
                attributeCode += attributeValue + "}";

                if (!everyAttributeIsSet) {
                    continue;
                }
            } else if (attribute.dataType === "std::string") {
                attributeCode = "\"" + block.getFieldValue('attribute' + i) + "\"";
                if (attributeCode === "not defined") {
                    continue;
                }
            } else
            {
                attributeCode = block.getFieldValue('attribute' + i);
                if (attributeCode === "not defined") {
                    continue;
                }
            }

            skillCode += "std::dynamic_pointer_cast<kukadu::" + skillSelection.controller + ">(skill" + skillCounter + ")->set" + attribute.name + "(";
            skillCode += attribute.dataType == "string" ? "'" : "";
            skillCode += attributeCode;
            skillCode += attribute.dataType === "string" ? "'" : "";
            skillCode += ");\n";
        }

        skillCode += "\nskill" + skillCounter++ + "->execute();\n";

        skillCode += "\n";
    }
    var code = hardwareCode + skillCode;

    return code;
};

Blockly.cake['hardware'] = function (block) {
    var selectedHardwareOption = block.getFieldValue('HardwareOptions');
    var hardwareSelection = Blockly.cake.valueToCode(block, 'Further', Blockly.cake.ORDER_ATOMIC);
    var code = selectedHardwareOption;

    if (hardwareSelection != "") {
        code += ", " + hardwareSelection;
    }

    return [code, Blockly.cake.ORDER_NONE];
};