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
    Blockly.cake.definitions_['include_kukaduImport_string'] = "#include<kukadu/kukadu.hpp>";

    // Define a procedure with a return value.
    var skillCode = Blockly.cake.statementToCode(block, 'STACK');

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
    roscode += "std::string executionType = \"Simulate\";\n" +
        "if(argc > 1)\n" +
        "\texecutionType = args[1];\n" +
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

    var createSkill = skillName + " skill(storage, {";
    for(var i = 0; i < hardwareids.length; i++){
        if(i < hardwareids.length-1){
            createSkill += hardwareids[i] + ", ";
        } else {
            createSkill += hardwareids[i];
        }
    }

	createSkill += "});\n";
    var executeSkill = "skill.execute();\n";

    if (installSkill === 'TRUE') {
        createSkill = "kukadu::" + createSkill;
        installSkill = "try { skill.createSkillFromThis(\"" + skillName + "\"); } catch(kukadu::KukaduException& ex) {}\n";
    } else {
        Blockly.cake.definitions_['include_install_not_in_package'] = "#include <generated_graphical_programming/header.hpp>\n";
        installSkill = "\n";
    }

    var playingQueue = block.getFieldValue('PlayingQueueHardware');
    var playingHand = block.getFieldValue('PlayingHandHardware');
    var stateCount = block.getFieldValue('StateCount');
    var behaviour = block.getFieldValue('BehaviourMode');
    var codeClass = new CodeClass(skillName, skillCode, behaviour, playingQueue, playingHand, stateCount);

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
        allDefs.replace(/\n\n+/g, '\n\n').replace(/\n*$/, '\n') + installHardware + createSkill + installSkill + executeSkill + returnValue + '}\n\n\n\n' + codeClass.generateClass();
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

function CodeClass(name, code, behaviour, playingQueue, playingHand, stateCount) {
    this.name = name;
    this.code = code;
    this.behaviour = behaviour;
    this.playingQueue = playingQueue;
    this.playingHand = playingHand;
    this.stateCount = stateCount;

    this.generateClass = function () {
        var splitCode = "//Skillheader for Skill\n" +
            "#ifndef KUKADU_GENERATED_SKILLS_" + getCurrentSkillName().toUpperCase() + "_H\n" +
            "#define KUKADU_GENERATED_SKILLS_" + getCurrentSkillName().toUpperCase() + "_H\n\n" +
            Blockly.cake.definitions_['include_kukadu_string'] + "\n";
            if (typeof Blockly.cake.definitions_['include_PoseEstimatorFactory_string'] != "undefined") {
                splitCode += Blockly.cake.definitions_['include_PoseEstimatorFactory_string'];
            }

            splitCode += "\n";

        switch (behaviour) {
            case 'Behaviour':
                splitCode += this.getBehaviourCode();
                break;
            case 'ComplexBehaviour':
                splitCode += this.getComplexBehaviourCode();
                break;
            case 'SensingBehaviour':
                splitCode += this.getSensingBehaviourCode();
                break;
        }

        return splitCode;
    };

    this.getBehaviourCode = function () {
        var skillHeaderContent = "";
        var newLine = "\n";

        if (isSkillInstalled()) {
            skillHeaderContent += "namespace kukadu {\n\t";
            newLine= "\n\t";
        }
        skillHeaderContent +=
            "class " + name + " : public kukadu::Controller {" + newLine +
            newLine +
            "private:" + newLine +
            newLine +
            "protected:" + newLine +
            newLine +
            "\tvirtual void createSkillFromThisInternal(std::string skillName);" + newLine +
            newLine +
            "public:" + newLine +
            newLine +
            "\t" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);" + newLine +
            newLine +
            "\tbool requiresGraspInternal();" + newLine +
            newLine +
            "\tbool producesGraspInternal();" + newLine +
            newLine +
            "\tstd::shared_ptr<kukadu::ControllerResult> executeInternal();" + newLine +
            newLine +
            "\tstd::string getClassName();" + newLine +
            newLine +
            "};\n";
        if (isSkillInstalled()) {
            skillHeaderContent += "}";
        }


        var skillHeader = skillHeaderContent;

        skillHeader += "\n\n#endif\n";

        var skillImplementation = "//Skillimplementation for Skill:\n";
        if (isSkillInstalled()) {
            skillImplementation += "#include <kukadu/generated_skills/" + name + ".hpp>\n" + previousIncludes + "\n namespace kukadu {\n\t";
        } else {
            skillImplementation += "#include <generated_graphical_programming/header.hpp>\n" + previousIncludes + "\n\n";
        }

        skillImplementation += name + "::" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)\n" +
            " : Controller(storage, \"" + name + "\", hardware, 0.01) {" + newLine +
            newLine +
            "}" + newLine +
            newLine +
            "bool " + name + "::requiresGraspInternal() {" + newLine +
            "\treturn false;" + newLine +
            "}" + newLine +
            newLine +
            "bool " + name + "::producesGraspInternal() {" + newLine +
            "\treturn false;" + newLine +
            "}" + newLine +
            newLine +
            "std::shared_ptr<kukadu::ControllerResult> " + name + "::executeInternal() {" + newLine +
            this.code + newLine +
            "\treturn nullptr;" + newLine +
            "}\n" + newLine +
            newLine +
            "std::string " + name + "::getClassName() {" + newLine +
            "\treturn \"" + name + "\";" + newLine +
            "}" + newLine +
            newLine +
            "void " + name + "::createSkillFromThisInternal(std::string skillName) {" + newLine +
            "\t// nothing to do" + newLine +
            "}\n";

        if (isSkillInstalled()) {
            skillImplementation += "}\n\n\n\n\n";
        }

        return skillHeader + skillImplementation;
    };

    this.getSensingBehaviourCode = function () {

        var skillHeaderContent = "";
        var newLine= "\n";
        if (isSkillInstalled()) {
            skillHeaderContent += "namespace kukadu {\n\t";
            newLine = "\n\t";
        }
        skillHeaderContent +=
            "class " + name + " : public kukadu::SensingController {" + newLine +
            newLine +
            "private:" + newLine +
                "    std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware;" + newLine +
                "    virtual std::vector<KUKADU_SHARED_PTR< kukadu::Hardware> > getUsedHardware();" + newLine +
            newLine +
            "protected:" + newLine +
            newLine +
            "    virtual bool requiresGraspInternal();" + newLine +
            "    virtual bool producesGraspInternal();" + newLine +
            newLine +
            "public:" + newLine +
            newLine +
            "    " + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);" +
            newLine +
            "    virtual void prepare();" + newLine +
            "    virtual void cleanUp();" + newLine +
            "    virtual void performCore();" + newLine +
            newLine +
            "    virtual std::string getClassName();" +
            newLine +
            "    virtual KUKADU_SHARED_PTR<kukadu::SensingController> clone();" + newLine +
            "};\n";
        if (isSkillInstalled()) {
            skillHeaderContent += "}";
        }


        var skillHeader = skillHeaderContent;

        skillHeader += "\n\n#endif\n";


        var skillImplementation = "//Skillimplementation for Skill:\n";
        if (isSkillInstalled()) {
            skillImplementation += "#include <kukadu/generated_skills/" + name + ".hpp>\n" + previousIncludes + "\n namespace kukadu {\n\t";
        } else {
            skillImplementation += "#include <generated_graphical_programming/header.hpp>\n" + previousIncludes + "\n\n";
        }

        var splitCodeForSensing = this.getSplitCode();

        skillImplementation +=
            "template<typename T > KUKADU_SHARED_PTR< T > retrieveHardwareWithId(std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > list, std::string instanceName) {" + newLine +
            "\tfor(auto& hw : list) {" + newLine +
            "\t\tif(hw->getHardwareInstanceName() == instanceName)" + newLine +
            "\t\t\treturn KUKADU_DYNAMIC_POINTER_CAST< T >(hw);" + newLine +
            "\t}" + newLine +
            "\tthrow kukadu::KukaduException(\"(utils) hardware to search is not in the list\");" + newLine +
            "}" + newLine +
            newLine +
            name + "::" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)" + newLine +
            " : SensingController(storage, kukadu::SkillFactory::get().getGenerator(), kukadu::SensingController::HAPTIC_MODE_CLASSIFIER, " + newLine +
            "\"" + name + "\", {retrieveHardwareWithId <kukadu::ControlQueue> (hardware, \"" + playingQueue + "\")},\n" +
            "    {retrieveHardwareWithId <kukadu::GenericHand>(hardware, \"" + playingHand + "\")}, \"/tmp/\", 1.0) {" + newLine +
            "\tthis->hardware = hardware;" + newLine +
            "}" + newLine +
            newLine +
            "std::vector<KUKADU_SHARED_PTR< kukadu::Hardware> > " + name + "::getUsedHardware() {" + newLine +
            "\treturn hardware;" + newLine +
            "}" + newLine +
            newLine +
            "bool " + name + "::requiresGraspInternal() {" + newLine +
            "\treturn false;" + newLine +
            "}" + newLine +
            newLine +
            "bool " + name + "::producesGraspInternal() {" + newLine +
            "\treturn false;" + newLine +
            "}" + newLine +
            newLine +
            "void " + name + "::prepare() {" + newLine +
            splitCodeForSensing[0] + newLine +
            "}" + newLine +
            "void " + name + "::cleanUp() {" + newLine +
            splitCodeForSensing[2] + newLine +
            "}" + newLine +
            "void " + name + "::performCore() {" + newLine +
            splitCodeForSensing[1] + newLine +
            "}" + newLine +
            newLine +
            "std::string " + name + "::getClassName() {" + newLine +
            "\treturn \"" + name + "\";" + newLine +
            "}" + newLine +
            newLine +
            "KUKADU_SHARED_PTR<kukadu::SensingController> " + name + "::clone() {" + newLine +
            "\treturn std::make_shared<" + name + ">(getStorage(), getUsedHardware());" + newLine +
            "}\n";

        if (isSkillInstalled()) {
            skillImplementation += "}\n\n\n\n\n";
        }

        return skillHeader + skillImplementation;
    };

    this.getComplexBehaviourCode = function () {
        var skillHeaderContent = "";
        var newLine = "\n";

        if (isSkillInstalled()) {
            skillHeaderContent += "namespace kukadu {\n\t";
            newLine= "\n\t";
        }
        skillHeaderContent +=
            "class " + name + " : public kukadu::ComplexController {" + newLine +
            newLine +
            "private:" + newLine +
            newLine +
            "protected:" + newLine +
            newLine +
            "    virtual bool requiresGraspInternal();" + newLine +
            "    virtual bool producesGraspInternal();" + newLine +
            newLine +
            "    virtual void createSkillFromThisInternal(std::string skillName);" + newLine +
            newLine +
            "public:" + newLine +
            newLine +
            "    " + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);" + newLine +
            newLine +
            "    virtual void executeComplexAction();" + newLine +
            "    virtual void cleanupAfterAction();" + newLine +
            newLine +
            "    virtual std::string getClassName();" + newLine +
            "    int getStateCount(const std::string& sensingName);" + newLine +
            "    void prepareNextState(KUKADU_SHARED_PTR<kukadu::SensingController> cont, int currentStateIdx);" + newLine +
            "    virtual double getSimulatedReward(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip, KUKADU_SHARED_PTR<kukadu::Clip> stateClip, KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip);" + newLine +
            "    virtual KUKADU_SHARED_PTR<kukadu::Clip> computeGroundTruthTransition(KUKADU_SHARED_PTR<kukadu::Clip> sensingClip, KUKADU_SHARED_PTR<kukadu::Clip> stateClip, KUKADU_SHARED_PTR<kukadu::Clip> actionClip);" + newLine +
                newLine +
            "};";
        if (isSkillInstalled()) {
            skillHeaderContent += "}";
        }


        var skillHeader = skillHeaderContent;

        skillHeader += "\n\n#endif\n";


        var skillImplementation = "//Skillimplementation for Skill:\n";
        if (isSkillInstalled()) {
            skillImplementation += "#include <kukadu/generated_skills/" + name + ".hpp>\n" + previousIncludes + "\n namespace kukadu {\n\t";
        } else {
            skillImplementation += "#include <generated_graphical_programming/header.hpp>\n" + previousIncludes + "\n\n";
        }

        skillImplementation += name + "::" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)" + newLine +
            "    : ComplexController(storage, \"" + name + "\", hardware, kukadu::resolvePath(\"$KUKADU_HOME/skills/" + name + "\"), kukadu::SkillFactory::get().getGenerator(), kukadu::SkillFactory::get().loadSkill(\"nothing\", {})) {\n" +
            "}" + newLine +
            newLine +
            "void " + name + "::cleanupAfterAction() {" + newLine +
            newLine +
            "}" + newLine +
            newLine +
            "bool " + name + "::requiresGraspInternal() {" + newLine +
            "    return false;" + newLine +
            "}" + newLine +
            newLine +
            "bool " + name + "::producesGraspInternal() {" + newLine +
            "    return false;" + newLine +
            "}" + newLine +
            newLine +
            "std::string " + name + "::getClassName() {" + newLine +
            "    return \"" + name + "\";" + newLine +
            "}" + newLine +
            newLine +
            "void " + name + "::createSkillFromThisInternal(std::string skillName) {" + newLine +
            "}" + newLine +
            newLine +
            "int " + name + "::getStateCount(const std::string& sensingName) {" + newLine +
            "\treturn " + stateCount + ";" + newLine +
            "}" + newLine +
            newLine +
            "double " + name + "::getSimulatedReward(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip," + newLine +
            "KUKADU_SHARED_PTR<kukadu::Clip> stateClip," + newLine +
            "\t\tKUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip) {" + newLine +
            newLine +
            "\t\tthrow kukadu::KukaduException(\"(" + name + ") simulated reward not supported for generated skills\");" + newLine +
            "}" + newLine +
            newLine +
            "KUKADU_SHARED_PTR<kukadu::Clip> " + name + "::computeGroundTruthTransition(KUKADU_SHARED_PTR<kukadu::Clip> sensingClip, KUKADU_SHARED_PTR<kukadu::Clip> stateClip, KUKADU_SHARED_PTR<kukadu::Clip> actionClip) {\n" +
            newLine +
            "\tthrow kukadu::KukaduException(\"(" + name + ") ground truth not available for generated skills\");" + newLine +
            newLine +
            "}" + newLine +
            newLine +
            "void " + name + "::prepareNextState(KUKADU_SHARED_PTR<kukadu::SensingController> cont, int currentStateIdx) {" + newLine +
            "    // nothing to do" + newLine +
            "}" + newLine +
            newLine +
            "void " + name + "::executeComplexAction() {" + newLine +
            this.code + newLine +
            "}\n";

        if (isSkillInstalled()) {
            skillImplementation += "}\n\n\n\n\n";
        }

        return skillHeader + skillImplementation;
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
