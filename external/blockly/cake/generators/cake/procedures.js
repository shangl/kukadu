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
    var skillCode = 'auto hardwareFactory = kukadu::HardwareFactory::get();\n' + Blockly.cake.statementToCode(block, 'STACK');

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
        "kukadu::StorageSingleton& storage = kukadu::StorageSingleton::get();\n" +
        "auto& hardwareFactory = kukadu::HardwareFactory::get();\n\n";

    var executionType = block.getFieldValue('ExecutionMode');

    var installHardware = "";
    var hardwareids = [];
    for (var key in Blockly.cake.neededHardware_) {

        hardwareids.push(key);
        installHardware += Blockly.cake.neededHardware_[key];
    }


    var skillName = block.getFieldValue('newSkillName');
    Blockly.cake.activeSkill_ = skillName;
    var executeSkill = skillName + " skill(storage, {" + hardwareids + "});\nskill.execute();\n";

    var installSkill = block.getFieldValue('CheckBoxInstallSkill');

    if (installSkill === 'TRUE') {
        installSkill = "try { skill.createSkillFromThis(\"" + skillName + "\"); } catch(kukadu::KukaduException& ex) {}\n";
    } else {
        installSkill = "\n";
    }

    var codeClass = new CodeClass(skillName, skillCode);

    var importsForMain = "#include \"../include/" + skillName + ".hpp\"\n"
        + "#include \"" + skillName + ".cpp\"\n\n";

    var funcName = 'main';
    var returnValue = "return EXIT_SUCCESS;\n";
    var returnType = 'int';
    roscode = Blockly.cake.prefixLines(roscode, Blockly.cake.INDENT);
    installHardware = Blockly.cake.prefixLines(installHardware, Blockly.cake.INDENT);
    executeSkill = Blockly.cake.prefixLines(executeSkill, Blockly.cake.INDENT);
    installSkill = Blockly.cake.prefixLines(installSkill, Blockly.cake.INDENT);
    returnValue = Blockly.cake.prefixLines(returnValue, Blockly.cake.INDENT);

    var code = importsForMain + returnType + ' ' + funcName + '(' + typePlusArgs.join(', ') + ') {' + "\n" +
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

function CodeClass(name, code) {
    this.name = name;
    this.code = code;

    this.generateClass = function () {
        var skillHeader = "//Skillheader for Skill\n" +
            Blockly.cake.definitions_['include_cake_string'] + "\n";

        var skillHeaderContent = "class " + name + " : public kukadu::Controller {\n" +
            "\n" +
            "private:\n" +
            "\n" +
            "\tstd::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware;\n" +
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
            "};";


        skillHeader += skillHeaderContent;


        var skillImplementation = "//Skillimplementation for Skill:\n" +
            "#import <../include/" + name + ".hpp>\n";

        skillImplementation += name + "::" + name + "(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)\n" +
            " : Controller(storage, \"" + name + "\", hardware, 0.01) {\n" +
            "\n" +
            "\tthis->hardware = hardware;\n" +
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
            "std::shared_ptr<kukadu::ControllerResult> " + name + "::executeInternal() {\n" + this.code + "\n" +
            "}\n" +
            "\n" +
            "std::string " + name + "::getClassName() {\n" +
            "\treturn \"" + name + "\";\n" +
            "}\n" +
            "\n" +
            "void " + name + "::createSkillFromThisInternal(std::string skillName) {\n" +
            "\t// nothing to do\n" +
            "}\n\n\n\n\n";

        return skillHeader + skillImplementation;
    }
}