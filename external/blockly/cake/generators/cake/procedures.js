/**
 * @license
 * Visual Blocks Language
 *
 * Copyright 2012 Google Inc.
 * https://blockly.googlecode.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview Generating cake for procedure blocks.
 * @author fraser@google.com (Neil Fraser)
 */
'use strict';

goog.provide('Blockly.cake.procedures');

goog.require('Blockly.cake');

Blockly.cake['main_block'] = function (block) {
    // Define a procedure with a return value.
    var skillCode = Blockly.cake.statementToCode(block, 'STACK') + "\nreturn nullptr;";

    if (Blockly.cake.STATEMENT_PREFIX) {
        skillCode = Blockly.cake.prefixLines(
            Blockly.cake.STATEMENT_PREFIX.replace(/%1/g,
                '\'' + block.id + '\''), Blockly.cake.INDENT) + skillCode;
    }

    if (Blockly.cake.INFINITE_LOOP_TRAP) {
        skillCode = Blockly.cake.INFINITE_LOOP_TRAP.replace(/%1/g,
            '\'' + block.id + '\'') + skillCode;
    }


    var args = [];
    var argTypes = [];
    var typePlusArgs = [];
    for (var x = 0; x < block.arguments_.length; x++) {
        args[x] = Blockly.cake.variableDB_.getName(block.arguments_[x],
            Blockly.Variables.NAME_TYPE);
        argTypes[x] = block.types_[x];
        typePlusArgs[x] = argTypes[x] + ' ' + args[x];
    }

    var rand = [];
    var time = [];
    for (var name in Blockly.cake.times_) {
        var def = Blockly.cake.times_[name];
        var nameSrand = 'srand';
        var nameTime = 'time';
        var preDef;
        if (name.match(nameSrand)) {
            if (def[0] == 'main_block') {
                preDef = Blockly.cake.prefixLines(def[1], Blockly.cake.INDENT);
                rand.push(preDef);
            }
        }
        else if (name.match(nameTime)) {
            if (def[0] == 'main_block') {
                preDef = Blockly.cake.prefixLines(def[1], Blockly.cake.INDENT);
                time.push(preDef);
            }
        }
    }

    if (rand.length) {
        var allDefs = rand.join('\n') + '\n' + time.join('\n');
    }
    else {
        var allDefs = time.join('\n');
    }

    var roscode = "ros::init(argc, args, \"kukadu\");\n" +
        "ros::NodeHandle* node = new ros::NodeHandle();\n" +
        "usleep(1e6);\n" +
        "ros::AsyncSpinner spinner(10);\n" +
        "spinner.start();\n" +
        "auto& storage = kukadu::StorageSingleton::get();\n" +
        "auto& hardwareFactory = kukadu::HardwareFactory::get();\n";

    Blockly.cake.executionMode = block.getFieldValue('ExecutionMode');
    roscode += "std::string executionType = args[1];\n" +
        "hardwareFactory.setSimulation(executionType == \"Simulate\");"

    var installHardware = "";
    var hardwareids = [];
    for (var key in Blockly.cake.neededHardware_) {
        hardwareids.push(key);
        installHardware += Blockly.cake.neededHardware_[key];
    }


    var skillName = block.getFieldValue('newSkillName');
    Blockly.cake.activeSkill_ = skillName;
    var installSkill = block.getFieldValue("CheckBoxInstallSkill");
    Blockly.cake.installSkill = installSkill;

    var executeSkill = skillName + " skill(storage, {" + hardwareids + "});\nskill.execute();\n";

    if (installSkill === 'TRUE') {
        executeSkill = "kukadu::" + executeSkill;
        installSkill = "try { skill.createSkillFromThis(\"" + skillName + "\"); } catch(kukadu::KukaduException& ex) {}\n";
    } else {
        Blockly.cake.definitions_['include_install_not_in_package'] = "#include <generated_graphical_programming/header.hpp>\n";
        installSkill = "\n";
    }

    var behaviour = block.getFieldValue('BehaviourMode');
    var codeClass = new CodeClass(skillName, skillCode, behaviour);

    var funcName = 'main';
    var returnValue = "hardwareFactory.stopAllCreatedHardware();\nkukadu::ModuleUsageSingleton::get().stopStatisticsModule();\nstorage.waitForEmptyCache();\nreturn EXIT_SUCCESS;\n";
    var returnType = 'int';
    roscode = Blockly.cake.prefixLines(roscode, Blockly.cake.INDENT);
    installHardware = Blockly.cake.prefixLines(installHardware, Blockly.cake.INDENT);
    executeSkill = Blockly.cake.prefixLines(executeSkill, Blockly.cake.INDENT);
    installSkill = Blockly.cake.prefixLines(installSkill, Blockly.cake.INDENT);
    returnValue = Blockly.cake.prefixLines(returnValue, Blockly.cake.INDENT);

    var code = returnType + ' ' + funcName + '(' + typePlusArgs.join(', ') + ') {' + "\n" +
        roscode +
        allDefs.replace(/\n\n+/g, '\n\n').replace(/\n*$/, '\n') + installHardware + executeSkill + installSkill + returnValue + '}\n\n\n\n' + codeClass.generateClass();
    code = Blockly.cake.scrub_(block, code);

    Blockly.cake.definitions_[funcName] = code;
    return null;
};

Blockly.cake['procedures_return'] = function (block) {
    var returnValue = block.getFieldValue('VALUE');
    if (returnValue) {
        return 'return ' + returnValue + ';\n';
    }
    else {
        return 'return 0;\n';
    }
};

Blockly.cake['procedures_defreturn'] = function (block) {
    // Define a procedure with a return value.
    var funcName = Blockly.cake.variableDB_.getName(
        block.getFieldValue('NAME'), Blockly.Procedures.NAME_TYPE);
    var branch = Blockly.cake.statementToCode(block, 'STACK');
    if (Blockly.cake.STATEMENT_PREFIX) {
        branch = Blockly.cake.prefixLines(
            Blockly.cake.STATEMENT_PREFIX.replace(/%1/g,
                '\'' + block.id + '\''), Blockly.cake.INDENT) + branch;
    }
    if (Blockly.cake.INFINITE_LOOP_TRAP) {
        branch = Blockly.cake.INFINITE_LOOP_TRAP.replace(/%1/g,
            '\'' + block.id + '\'') + branch;
    }
    var returnValue = Blockly.cake.valueToCode(block, 'RETURN',
        Blockly.cake.ORDER_NONE) || '';
    if (returnValue) {
        returnValue = '  return ' + returnValue + ';\n';
    }
    else {
        returnValue = '  return 0;\n';
    }
    var typePlusArgs = Blockly.Procedures.getTypePlusArgs(block);

    var rand = [];
    var time = [];
    for (var name in Blockly.cake.times_) {
        var def = Blockly.cake.times_[name];
        var nameSrand = 'srand';
        var nameTime = 'time';
        var preDef;
        if (name.match(nameSrand)) {
            if (def[0] == funcName) {
                preDef = Blockly.cake.prefixLines(def[1], Blockly.cake.INDENT);
                rand.push(preDef);
            }
        }
        else if (name.match(nameTime)) {
            if (def[0] == funcName) {
                preDef = Blockly.cake.prefixLines(def[1], Blockly.cake.INDENT);
                time.push(preDef);
            }
        }
    }
    if (rand.length) {
        var allDefs = rand.join('\n') + '\n' + time.join('\n');
    }
    else {
        var allDefs = time.join('\n');
    }

    var returnType = block.getFieldValue('TYPES');
    var returnDist = block.getFieldValue('DISTS');
    var returnSpec, code;
    if (returnDist == 'pointer') {
        returnSpec = block.getFieldValue('PSPECS');
        if (returnSpec == null) {
            returnSpec = '*';
        }
        code = returnType + returnSpec + ' ' + funcName + '(' + typePlusArgs.join(', ') + ') {\n' +
            allDefs.replace(/\n\n+/g, '\n\n').replace(/\n*$/, '\n') + branch + returnValue + '}';
    }
    else if (returnDist == 'array') {
        returnSpec = block.getFieldValue('ASPECS');
        if (returnSpec == null) {
            returnSpec = '[]';
        }
        code = returnType + returnSpec + ' ' + funcName + '(' + typePlusArgs.join(', ') + ') {\n' +
            allDefs.replace(/\n\n+/g, '\n\n').replace(/\n*$/, '\n') + branch + returnValue + '}';
    }
    else {
        code = returnType + ' ' + funcName + '(' + typePlusArgs.join(', ') + ') {\n' +
            allDefs.replace(/\n\n+/g, '\n\n').replace(/\n*$/, '\n') + branch + returnValue + '}';
    }
    code = Blockly.cake.scrub_(block, code);
    Blockly.cake.definitions_[funcName] = code;
    Blockly.cake.definitions_['Func_declare' + funcName] =
        returnType + ' ' + funcName + '(' + typePlusArgs.join(', ') + ');';
    if (Blockly.Blocks.checkLegalName(Blockly.Msg.PROCEDURES_ILLEGALNAME, funcName) == -1) {
        this.initName();
    }
    return null;
};

// Defining a procedure without a return value uses the same generator as
// a procedure with a return value.
Blockly.cake['procedures_defnoreturn'] = function (block) {
    // Define a procedure with a return value.
    var funcName = Blockly.cake.variableDB_.getName(
        block.getFieldValue('NAME'), Blockly.Procedures.NAME_TYPE);
    var branch = Blockly.cake.statementToCode(block, 'STACK');

    var rand = [];
    var time = [];
    for (var name in Blockly.cake.times_) {
        var def = Blockly.cake.times_[name];
        var nameSrand = 'srand';
        var nameTime = 'time';
        var preDef;
        if (name.match(nameSrand)) {
            if (def[0] == funcName) {
                preDef = Blockly.cake.prefixLines(def[1], Blockly.cake.INDENT);
                rand.push(preDef);
            }
        }
        else if (name.match(nameTime)) {
            if (def[0] == funcName) {
                preDef = Blockly.cake.prefixLines(def[1], Blockly.cake.INDENT);
                time.push(preDef);
            }
        }
    }
    if (rand.length) {
        var allDefs = rand.join('\n') + '\n' + time.join('\n');
    }
    else {
        var allDefs = time.join('\n');
    }

    if (Blockly.cake.STATEMENT_PREFIX) {
        branch = Blockly.cake.prefixLines(
            Blockly.cake.STATEMENT_PREFIX.replace(/%1/g,
                '\'' + block.id + '\''), Blockly.cake.INDENT) + branch;
    }
    if (Blockly.cake.INFINITE_LOOP_TRAP) {
        branch = Blockly.cake.INFINITE_LOOP_TRAP.replace(/%1/g,
            '\'' + block.id + '\'') + branch;
    }
    var returnValue = Blockly.cake.valueToCode(block, 'RETURN',
        Blockly.cake.ORDER_NONE) || '';
    if (returnValue) {
        returnValue = '  return ' + returnValue + ';\n';
    }
    var typePlusArgs = Blockly.Procedures.getTypePlusArgs(block);

    var code = 'void ' + funcName + '(' + typePlusArgs.join(', ') + ') {\n' +
        allDefs.replace(/\n\n+/g, '\n\n').replace(/\n*$/, '\n') + branch + returnValue + '}';
    code = Blockly.cake.scrub_(block, code);
    Blockly.cake.definitions_[funcName] = code;
    Blockly.cake.definitions_['Func_declare' + funcName] =
        'void ' + funcName + '(' + typePlusArgs.join(', ') + ');';
    if (Blockly.Blocks.checkLegalName(Blockly.Msg.PROCEDURES_ILLEGALNAME, funcName) == -1) {
        this.initName();
    }
    return null;
};

Blockly.cake['procedures_callreturn'] = function (block) {
    // Call a procedure with a return value.
    var funcName = Blockly.cake.variableDB_.getName(
        block.getFieldValue('NAME'), Blockly.Procedures.NAME_TYPE);
    var args = [];
    for (var x = 0; x < block.arguments_.length; x++) {
        args[x] = Blockly.cake.valueToCode(block, 'ARG' + x,
            Blockly.cake.ORDER_COMMA) || 'null';
    }
    var code = funcName + '(' + args.join(', ') + ')';
    return [code, Blockly.cake.ORDER_FUNCTION_CALL];
};

Blockly.cake['procedures_callnoreturn'] = function (block) {
    // Call a procedure with no return value.
    var funcName = Blockly.cake.variableDB_.getName(
        block.getFieldValue('NAME'), Blockly.Procedures.NAME_TYPE);
    var args = [];
    for (var x = 0; x < block.arguments_.length; x++) {
        args[x] = Blockly.cake.valueToCode(block, 'ARG' + x,
            Blockly.cake.ORDER_COMMA) || 'null';
    }
    var code = funcName + '(' + args.join(', ') + ');\n';
    return code;
};

function CodeClass(name, code, behaviour) {
    this.name = name;
    this.code = code;
    this.behaviour = behaviour;

    this.generateClass = function () {
        var splitCode = "//Skillheader for Skill\n" +
            "#ifndef KUKADU_GENERATED_SKILLS_" + getCurrentSkillName().toUpperCase() + "_H\n" +
            "#define KUKADU_GENERATED_SKILLS_" + getCurrentSkillName().toUpperCase() + "_H\n\n" +
            Blockly.cake.definitions_['include_cake_string'] + "\n";

        switch (behaviour) {
            case 'Behaviour':
                splitCode += this.getBehaviourCode();
                break;
            case 'ComplexBehaviour':
                break;
            case 'SensingBehaviour':
                splitCode += this.getSensingBehaviourCode();
                break;
        }

        return splitCode;
    };

    this.getBehaviourCode = function () {
        var skillHeaderContent = "";
        if (isSkillInstalled()) {
            skillHeaderContent += "namespace kukadu {\n\t";
        }
        skillHeaderContent +=
            "class " + name + " : public kukadu::Controller {\n" +
            "\n" +
            "private:\n" +
            "\n" +
            "protected:\n" +
            "\n" +
            "\tvirtual void createSkillFromThisInternal(std::string skillName);\n" +
            "\n" +
            "public:\n" +
            "\n" +
            "\t" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);\n" +
            "\n" +
            "\tbool requiresGraspInternal();\n" +
            "\n" +
            "\tbool producesGraspInternal();\n" +
            "\n" +
            "\tstd::shared_ptr<kukadu::ControllerResult> executeInternal();\n" +
            "\n" +
            "\tstd::string getClassName();\n" +
            "\n" +
            "};\n";
        if (isSkillInstalled()) {
            skillHeaderContent += "}";
        }


        var skillHeader = skillHeaderContent;

        skillHeader += "\n\n#endif\n";


        var skillImplementation = "//Skillimplementation for Skill:\n";
        if (isSkillInstalled()) {
            skillImplementation += "#include <kukadu/generated_skills/" + name + ".hpp>\nnamespace kukadu {\n\t";
        } else {
            skillImplementation += "#include <generated_graphical_programming/header.hpp>\n\n\n";
        }

        skillImplementation += name + "::" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)\n" +
            " : Controller(storage, \"" + name + "\", hardware, 0.01) {\n" +
            "\n" +
            "}\n" +
            "\n" +
            "bool " + name + "::requiresGraspInternal() {\n" +
            "\treturn false;\n" +
            "}\n" +
            "\n" +
            "bool " + name + "::producesGraspInternal() {\n" +
            "\treturn false;\n" +
            "}\n" +
            "\n" +
            "std::shared_ptr<kukadu::ControllerResult> " + name + "::executeInternal() {\n" +
            Blockly.cake.prefixLines(this.code, Blockly.cake.INDENT) + "\n" +
            "}\n" +
            "\n" +
            "std::string " + name + "::getClassName() {\n" +
            "\treturn \"" + name + "\";\n" +
            "}\n" +
            "\n" +
            "void " + name + "::createSkillFromThisInternal(std::string skillName) {\n" +
            "\t// nothing to do\n" +
            "}\n";

        if (isSkillInstalled()) {
            skillImplementation += "}\n\n\n\n\n";
        }

        return skillHeader + skillImplementation;
    };

    this.getSensingBehaviourCode = function () {

        var skillHeaderContent = "";
        if (isSkillInstalled()) {
            skillHeaderContent += "namespace kukadu {\n\t";
        }
        skillHeaderContent +=
            "class " + name + " : public kukadu::SensingController {\n" +
            "\n" +
            "private:\n" +
            "\n" +
            "protected:\n" +
            "\n" +
            "    virtual bool requiresGraspInternal();\n" +
            "    virtual bool producesGraspInternal();\n" +
            "\n" +
            "    virtual void createSkillFromThisInternal(std::string skillName);\n" +
            "\n" +
            "public:\n" +
            "\n" +
            name + "(StorageSingleton& storage, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, string caption,\n" +
            "                                         std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, vector<KUKADU_SHARED_PTR<GenericHand> > hands,\n" +
            "                                         std::string tmpPath, int simClassificationPrecision)" +
            "\n" +
            "    virtual void prepare();\n" +
            "    virtual void cleanUp();\n" +
            "    virtual void performCore();\n" +
            "\n" +
            "    virtual std::string getClassName();" +
            "\n" +
            "};\n";
        if (isSkillInstalled()) {
            skillHeaderContent += "}";
        }


        var skillHeader = skillHeaderContent;

        skillHeader += "\n\n#endif\n";


        var skillImplementation = "//Skillimplementation for Skill:\n";
        if (isSkillInstalled()) {
            skillImplementation += "#include <kukadu/generated_skills/" + name + ".hpp>\nnamespace kukadu {\n\t";
        } else {
            skillImplementation += "#include <generated_graphical_programming/header.hpp>\n\n\n";
        }

        var splitCodeForSensing = this.getSplitCode();

        skillImplementation += name + "::" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)\n" +
            " : SensingController(storage, \"" + name + "\", hardware, 0.01) {\n" +
            "\n" +
            "}\n" +
            "\n" +
            "bool " + name + "::requiresGraspInternal() {\n" +
            "\treturn false;\n" +
            "}\n" +
            "\n" +
            "bool " + name + "::producesGraspInternal() {\n" +
            "\treturn false;\n" +
            "}\n" +
            "\n" +
            "void " + name + "::prepare() {\n" +
            splitCodeForSensing[0] + "\n" +
            "}\n" +
            "void " + name + "::cleanUp() {\n" +
            splitCodeForSensing[1] + "\n" +
            "}\n" +
            "void " + name + "::performCore() {\n" +
            splitCodeForSensing[2] + "\n" +
            "}\n" +
            "\n" +
            "std::string " + name + "::getClassName() {\n" +
            "\treturn \"" + name + "\";\n" +
            "}\n" +
            "\n" +
            "}\n";

        if (isSkillInstalled()) {
            skillImplementation += "}\n\n\n\n\n";
        }

        return skillHeader + skillImplementation;
    };

    this.getComplexBehaviourCode = function () {

    };

    this.getSplitCode = function () {
        var code = [];
        var codeBlocks = this.code.split("//StartCoreHere");
        code.push(codeBlocks[0]);
        codeBlocks = codeBlocks[1].split("//StartCleanupHere");
        code.push(codeBlocks[0]);
        code.push(codeBlocks[1]);

        return code;
    };
}
