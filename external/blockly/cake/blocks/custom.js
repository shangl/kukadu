'use strict';

var loading = false;
var availableFiles = "";
var previousIncludes = "";

goog.provide('Blockly.Blocks.custom');
goog.require('Blockly.Blocks');

function set_available_files(files) {
    availableFiles = files.split(';');

}

function setPreviousIncludes(prevIncludes) {
    previousIncludes = "";
    var splittedPrevIncludes = prevIncludes.split(",");
    $(splittedPrevIncludes).each(function (idx, el) {
        previousIncludes += "#include <kukadu/generated_skills/" + el + ">\n";
    });
}

Blockly.Blocks['startCore'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Here starts Code of Core for Sensing");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip('Start Core');
        this.setHelpUrl('');
    }
};

Blockly.Blocks['startCleanup'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Here starts Code of Cleanup for Sensing");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip('Start Core');
        this.setHelpUrl('');
    }
};

Blockly.Blocks['objectposition'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Load Objectposition");
        this.appendDummyInput()
            .appendField("VariableName: ")
            .appendField(new Blockly.FieldTextInput("variableName"), "VariableName");
        this.appendDummyInput()
            .appendField("Object to load: ")
            .appendField(new Blockly.FieldTextInput("something"), "ObjectType");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip('Object Position');
        this.setHelpUrl('');
    },

    /**
     * Return 'variables'.
     */
    getDist: function () {
        return 'c';
    },
    /**
     * Return Variable's Scope
     */
    getScope: function () {
        if (this.getSurroundParent()) {
            if (this.getSurroundParent().getName) {
                return this.getSurroundParent().getName();
            }
        }
    },
    /**
     * Return Variable's Scope
     */
    getSpec: function () {
        return null;
    },
    /**
     * Return this block's position
     */
    getPos: function () {
        return this.getRelativeToSurfaceXY().y;
    },
    /**
     * Return all variables's types referenced by this block.
     * @return {!Array.<string>} List of variable types.
     * @this Blockly.Block
     */
    getTypes: function () {
        return ['double', 'double', 'double', 'double', 'double', 'double', 'double'];
    },
    /**
     * Return all variables referenced by this block.
     * @return {!Array.<string>} List of variable names.
     * @this Blockly.Block
     */
    getVars: function () {
        var initialName = this.getFieldValue('VariableName');

        return [initialName + ".orientation.x",
            initialName + ".orientation.y",
            initialName + ".orientation.z",
            initialName + ".orientation.w",
            initialName + ".position.x",
            initialName + ".position.y",
            initialName + ".position.z"];
    },
    /**
     * Return all variables referenced by this block.
     * @return {!Array.<string>} List of variable names.
     * @this Blockly.Block
     */
    getDeclare: function () {
        return this.getVars();
    },
    /**
     * Notification that a variable is renaming.
     * If the name matches one of this block's variables, rename it.
     * @param {string} oldName Previous name of variable.
     * @param {string} newName Renamed variable.
     * @this Blockly.Block
     */
    renameVar: function (oldName, newName) {
        if (Blockly.Names.equals(oldName, this.getFieldValue('VariableName'))) {
            this.setFieldValue(newName, 'VariableName');
        }
    },

    //when the block is changed,
    onchange: function () {
        Blockly.Blocks.requireInFunction(this);
    }
};

function getRequiredHardware() {
    var hardwareIds = "";
    var firstTime = true;
    for (var key in Blockly.cake.neededHardware_) {
        if (key != undefined && key != null) {
            if (firstTime == true)
                firstTime = false;
            else
                hardwareIds += ",";
            hardwareIds += key;
        }
    }
    return hardwareIds;
}

Blockly.Blocks['objectdimension'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Load Objectdimensions");
        this.appendDummyInput()
            .appendField("VariableName: ")
            .appendField(new Blockly.FieldTextInput("variableName"), "VariableName");
        this.appendDummyInput()
            .appendField("Object to load: ")
            .appendField(new Blockly.FieldTextInput("something"), "ObjectType");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip('Object Position');
        this.setHelpUrl('');
    },

    /**
     * Return 'array'.
     */
    getDist: function () {
        return 'a';
    },
    /**
     * Return Variable's Scope
     */
    getScope: function () {
        if (this.getSurroundParent()) {
            if (this.getSurroundParent().getName) {
                return this.getSurroundParent().getName();
            }
        }
    },
    /**
     * Return Variable's Scope
     */
    getSpec: function () {
        return [1, 3];
    },
    /**
     * Return this block's position
     */
    getPos: function () {
        return this.getRelativeToSurfaceXY().y;
    },
    /**
     * Return all variables's types referenced by this block.
     * @return {!Array.<string>} List of variable types.
     * @this Blockly.Block
     */
    getTypes: function () {
        return ['double', 'double', 'double'];
    },
    /**
     * Return all variables referenced by this block.
     * @return {!Array.<string>} List of variable names.
     * @this Blockly.Block
     */
    getVars: function () {
        var initialName = this.getFieldValue('VariableName');

        return [initialName];
    },
    /**
     * Return all variables referenced by this block.
     * @return {!Array.<string>} List of variable names.
     * @this Blockly.Block
     */
    getDeclare: function () {
        return this.getVars();
    },
    /**
     * Notification that a variable is renaming.
     * If the name matches one of this block's variables, rename it.
     * @param {string} oldName Previous name of variable.
     * @param {string} newName Renamed variable.
     * @this Blockly.Block
     */
    renameVar: function (oldName, newName) {
        if (Blockly.Names.equals(oldName, this.getFieldValue('VariableName'))) {
            this.setFieldValue(newName, 'VariableName');
        }
    },

    //when the block is changed,
    onchange: function () {
        Blockly.Blocks.requireInFunction(this);
    }
};

Blockly.Blocks['skillloader'] = {
    init: function () {
        this._hardwareIds = -1;
        this._skillId = -1;

        this.appendValueInput("HARDWARE")
            .setCheck("Hardware")
            .appendField('Hardware: ');
        this.appendDummyInput("SKILL")
            .appendField('Available Skills: ')
            .appendField(new Blockly.FieldDropdown([['No Elements available', '']]), "SkillOptions");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip('Skillloader');
        this.setHelpUrl('');
    },

    getCurrentHardware: function () {
        var input = this.getInput('HARDWARE');
        var target = null;

        if (input != null) {
            target = input.connection.targetConnection;
        }

        var hardwareNames = [];
        if (target != null) {
            hardwareNames = target.sourceBlock_.getSelectedHardware();
        }

        var hardwareInstances = [];
        for (var i = 0; i < hardwareNames.length; i++) {
            var instance = Databaseloader.hardwareMap[hardwareNames[i]];
            hardwareInstances.push(instance);
        }

        return hardwareInstances;
    },

    getCurrentSkill: function () {
        var blockvalue = this.getFieldValue('SkillOptions');
        return Databaseloader.skillMap[blockvalue];
    },

    onchange: function (event) {
        if (loading == false) {
            var addedHardwareList = this.getCurrentHardware();
            var hardwareKey = hardwareListToKeys(addedHardwareList);
            var chosenConfig = Databaseloader.roboConfigMap[hardwareKey];
            if (getKeysFromMap(addedHardwareList).length > 0 && hardwareKey !== this._hardwareIds) {    //if addedHardware Changed
                this._hardwareIds = hardwareKey;

                if (typeof chosenConfig == 'undefined') {
                    this.setNoSkillsAvailable();
                } else {
                    var availableSkills = Databaseloader.roboConfigToSkillMap[chosenConfig.id];
                    this.setAvailableSkills(availableSkills);
                }
            } else if (getKeysFromMap(addedHardwareList).length == 0 && hardwareKey !== this._hardwareIds) {
                this._hardwareIds = -1;
                var skillsInput = this.getInput('SKILL');
                if (skillsInput != null) {
                    skillsInput.fieldRow[1].menuGenerator_ = [];
                    skillsInput.fieldRow[1].setValue("No Elements available");
                }
            }

            var currentSkill = this.getCurrentSkill();
            if (typeof currentSkill != 'undefined' && this._skillId != currentSkill.id) {      //skill changed
                this.setInputsForSelectedSkill();
                this._skillId = this.getCurrentSkill().id;
            } else if (typeof currentSkill === 'undefined') {
                this.setInputsForSelectedSkill();
                this._skillId = -1;
            }
        }
    },

    setNoSkillsAvailable: function () {
        var skillsInput = this.getInput('SKILL');
        skillsInput.fieldRow[1].menuGenerator_ = [];
        skillsInput.fieldRow[1].setValue("No Elements available");
    },

    setAvailableSkills: function (availableSkills) {
        var skillsInput = this.getInput('SKILL');
        var skillsNameArray = [];

        var skills = getValuesFromMap(availableSkills);
        for (var i = 0; i < skills.length; i++) {
            var skill = skills[i];
            skillsNameArray.push([skill.name, skill.name]);
        }

        skillsInput.fieldRow[1].menuGenerator_ = skillsNameArray;

        if (skillsNameArray != undefined && skillsNameArray != null && skillsNameArray[0] != undefined && skillsNameArray[0] != null && skillsNameArray[0][0] != undefined && skillsNameArray[0][0] != null) {

            skillsInput.fieldRow[1].setValue(skillsNameArray[0][0]);

            this._skillId = this.getCurrentSkill();
            this.setInputsForSelectedSkill();

        } else {
            console.log("problem with setting the available skills");
            setFieldValueError = true;
        }

    },

    setInputsForSelectedSkill: function () {
        for (var i = this._inputsCount - 1; i >= 0; i--) {
            this.removeInput("attributes" + i);
        }
        var currentSkill = this.getCurrentSkill();

        if (typeof currentSkill === 'undefined') {
            this._inputsCount = 0;
            return;
        } else {
            var skillId = currentSkill.id;
            if(this._hardwareIds != undefined && Databaseloader.roboConfigMap[this._hardwareIds] != undefined) {
				var configId = Databaseloader.roboConfigMap[this._hardwareIds].id;
				if (configId != undefined && configId != null && Databaseloader.roboConfigToSkillMap[configId] != undefined && Databaseloader.roboConfigToSkillMap[configId] != null) {
					var skill = Databaseloader.roboConfigToSkillMap[configId][skillId];
					if (typeof skill !== 'undefined') {

						var attributes = getValuesFromMap(skill.getSetterFunctions());

						if (attributes != null) {
							var vectorIdOffset = 0;
							for (var i = 0; i < attributes.length; i++) {
								var attribute = attributes[i];
								var validator = Blockly.FieldTextInput.dummyValidator;
								if (attribute.dataType === "int") {
									validator = Blockly.FieldTextInput.integerValidator;
								} else if (attribute.dataType === "double") {
									validator = Blockly.FieldTextInput.floatValidator;
								}

								this.appendDummyInput("attributes" + i)
									.appendField(attribute.dataType + " " + attribute.name);
								var input = this.getInput("attributes" + i);

								if (attribute.dataType === "std::vector< double >" || attribute.dataType === "std::vector< int >" || attribute.dataType === "std::vector< string >") {

									switch (attribute.dataType) {
										case "std::vector< double >":
											validator = Blockly.FieldTextInput.floatValidator;
											break;
										case "std::vector< int >":
											validator = Blockly.FieldTextInput.integerValidator;
											break;
										case "std::vector< string >":
											validator = Blockly.FieldTextInput.dummyValidator;
											break;
									}

									var degOfFreedom = 0;
									var addedHardwareList = this.getCurrentHardware();
									var hardwareKey = hardwareListToKeys(addedHardwareList);
									var chosenConfig = Databaseloader.roboConfigMap[hardwareKey];

									for (var j = 0; j < chosenConfig.hardwareInOrder.length; j++) {
										var hardware = chosenConfig.hardwareInOrder[j];
										degOfFreedom = hardware.degOfFreedom > degOfFreedom ? hardware.degOfFreedom : degOfFreedom;
									}

									var vectorIdOffsetTmp = vectorIdOffset;
									for (var j = 0; j < degOfFreedom; j++) {
										var attributeId = i + vectorIdOffset;
										input.appendField(new Blockly.FieldTextInput(attribute.defaultValue + "", validator), "attribute" + attributeId);
										vectorIdOffset++;
									}
									vectorIdOffset = degOfFreedom > 0 ? vectorIdOffsetTmp + degOfFreedom - 1 : vectorIdOffset;
								} else {
									var attributeId = i + vectorIdOffset;
									input.appendField(new Blockly.FieldTextInput(attribute.defaultValue + "", validator), "attribute" + attributeId);
								}
							}

							this._inputsCount = attributes.length;
						}
					}
				} else {
					console.log("problem while loading skill " + skillId.toString());
					setFieldValueError = true;
				}
			}
		
        }
    }
};

Blockly.Blocks['hardware'] = {
    init: function () {
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown(Databaseloader.hardwareTupleArray, null, this), "HardwareOptions");
        this.appendValueInput("Further")
            .setCheck("Hardware");
        this.setInputsInline(true);
        this.setOutput(true, "Hardware");
        this.setColour(230);
        this.setTooltip("Here you can select the hardware for your robot.");
        this.setHelpUrl('');
    },

    getSelectedHardware: function () {
        var currentSelection = this.getFieldValue('HardwareOptions');
        var child = this.getInput('Further').connection.targetConnection;      //this is null or a hardwareBlock
        var selectedHardware = [];

        if (child != null) {
            selectedHardware = child.sourceBlock_.getSelectedHardware();        //this works because it is a hardware Block
        }

        return [currentSelection].concat(selectedHardware);
    },

    onchange: function (event) {
        Blockly.Blocks.requireInFunction(this);
    }
};

var Databaseloader = new function () {
    this.hardwareTupleArray = [];
    this.queuePlayingTupleArray = [];
    this.handPlayingTupleArray = [];
    this.roboConfigToSkillMap = {};
    this.skillMap = {};          //name to skill
    this.hardwareMap = {};       //name to hardware
    this.roboConfigMap = {};     //hardwareIds to configIds

    this.init = function (data) {
        Databaseloader.hardwareTupleArray = [];
        Databaseloader.queuePlayingTupleArray = [];
        Databaseloader.handPlayingTupleArray = [];
        Databaseloader.roboConfigToSkillMap = {};
        Databaseloader.skillMap = {};
        Databaseloader.hardwareMap = {};
        Databaseloader.roboConfigMap = {};
        this.attributePath = data['attributePath'];

        var idToHardware = {};
        for (var i = 0; i < data['hardwareInformation'].length; i++) {
            var hardwareEntry = data['hardwareInformation'][i];
            var hardwareInstance = new Hardware(hardwareEntry.hardwareId, hardwareEntry.hardwareName, hardwareEntry.degOfFreedom, hardwareEntry.classId);
            idToHardware[hardwareInstance.id] = hardwareInstance;
            Databaseloader.hardwareMap[hardwareInstance.name] = hardwareInstance;
        }

        for (var i = 0; i < data['roboConfigs'].length; i++) {
            var configEntry = data['roboConfigs'][i];
            var id = configEntry.id;
            var hwId = configEntry.hardwareId;
            var order = configEntry.order;
            var hardwareIdsInOrder = [];
            var hardwareInOrder = [];

            for (var j = 0; j < hwId.length; j++) {
                hardwareIdsInOrder[order[j] - 1] = hwId[j];     //-1 because order starts with 1
                hardwareInOrder[order[j] - 1] = idToHardware[hwId[j]];
            }

            Databaseloader.roboConfigMap[idsToKey(hardwareIdsInOrder)] = new RoboConfig(id, hardwareInOrder);
        }

        for (var i = 0; i < data['skillInformation'].length; i++) {
            var skillEntry = data['skillInformation'][i];
            var skillInstance = new Skill(skillEntry.id, skillEntry.skillName, skillEntry.controller);
            var configIds = skillEntry.configId;

            skillInstance.setConfigs(configIds);

            skillInstance.setSetterFunctions(Databaseloader.getSetterFunctions(skillInstance.name, skillInstance.controller));

            Databaseloader.skillMap[skillInstance.name] = skillInstance;
        }

        var skills = getValuesFromMap(Databaseloader.skillMap);
        for (var i = 0; i < skills.length; i++) {
            var skill = skills[i];
            var possibleConfigs = getValuesFromMap(skill.getConfigs());

            for (var j = 0; j < possibleConfigs.length; j++) {
                var setToAddSkill = Databaseloader.roboConfigToSkillMap[possibleConfigs[j]];

                if (typeof setToAddSkill === 'undefined') {
                    setToAddSkill = {};
                    Databaseloader.roboConfigToSkillMap[possibleConfigs[j]] = setToAddSkill;
                }

                setToAddSkill[skill.id] = skill;
            }
        }

        var hardwareArray = getValuesFromMap(idToHardware);
        for (var k = 0; k < hardwareArray.length; k++) {
            var hw = hardwareArray[k];
            Databaseloader.hardwareTupleArray.push([hw.name, hw.name]);
            if(hw.classId >= 100 && hw.classId < 200) {
                Databaseloader.handPlayingTupleArray.push([hw.name, hw.name]);
            } else if (hw.classId >= 200 && hw.classId < 300) {
                Databaseloader.queuePlayingTupleArray.push([hw.name, hw.name]);
            }
        }
    };

    function check_doc_exists(f) {
        return ($.inArray(f, availableFiles) >= 0);
    }

    function loadFile(filePath) {

        return $.ajax({
            type: "GET",
            url: filePath,
            dataType: "xml",
            async: false
        }).responseText;

    }

    this.getSetterFunctions = function (skill, controllerClass) {
        var setFunctions = {};
        var fileName = "classkukadu_1_1" + controllerClass + ".xml";
        var filePath = Databaseloader.attributePath + fileName;
        var nonFound = false;
        var xml = '';
        if (check_doc_exists(fileName)) {
            xml = loadFile(filePath);
        } else {
            var newFileName = "classkukadu_1_1skill_1_1" + controllerClass + ".xml";
            var newFilePath = Databaseloader.attributePath + newFileName;
            if (check_doc_exists(newFileName)) {
                xml = loadFile(newFilePath);
            } else {
                nonFound = true;
                console.log("neither " + fileName + " nor " + newFileName + " found");
            }
        }

        if (!nonFound) {

            var pubfunctionBlock = $(xml).find('sectiondef').filter(function () {
                return $(this).attr('kind') === "public-func";
            });

            $(pubfunctionBlock).find('memberdef').each(function () {
                var functionname = $(this).find('name').text();
                if (functionname.substring(0, 3) === "set") {
                    var argumentString = $(this).find('argsstring').text();

                    var regex = /\w+(<[\w:<>*,\s]+>\s\w+|[\w+\s:*]+)+/g;
                    var m;

                    while ((m = regex.exec(argumentString)) !== null) {
                        // This is necessary to avoid infinite loops with zero-width matches
                        if (m.index === regex.lastIndex) {
                            regex.lastIndex++;
                        }

                        var match = m[0];   //only get full match
                        var subStringIndex = match.lastIndexOf(" ");
                        var dataType = match.substr(0, subStringIndex);

                        var setFunction = new SetterFunction(functionname, dataType, "not defined");
                        setFunctions[setFunction.name] = setFunction;
                    }
                }

            });

        }

        return setFunctions;
    }
};

function eqSet(as, bs) {
    if (as.length !== bs.length) {
        return false;
    }

    var entriesOfAs = getKeysFromMap(as);
    var entriesOfBs = getKeysFromMap(bs);
    for (var i = 0; i < entriesOfAs.length; i++) {
        var a = entriesOfAs[i];
        if (bs.indexOf(a) === -1) {
            return false;
        }
    }
    return true;
}

function RoboConfig(id, hardwareInOrder) {
    this.id = id;
    this.hardwareInOrder = hardwareInOrder;
}

function Skill(id, name, controller) {
    this.id = id;
    this.name = name;
    this.controller = controller;

    this.setSetterFunctions = function (attributes) {
        this.attributes = attributes;
    };

    this.getSetterFunctions = function () {
        return this.attributes;
    };

    this.setConfigs = function (config) {
        this.roboConfig = config;
    };

    this.getConfigs = function () {
        return this.roboConfig;
    }
}

function Hardware(id, name, degOfFreedom, classId) {
    this.id = id;
    this.name = name;
    this.degOfFreedom = degOfFreedom;
    this.classId = classId;
}

function SetterFunction(name, dataType, defaultValue) {
    this.name = name;
    this.dataType = dataType;
    this.defaultValue = defaultValue;
}

function getKeysFromMap(map) {
    var keys = [];
    for (var key in map) {
        keys.push(key);
    }

    return keys;
}

function getValuesFromMap(map) {
    var values = [];
    for (var key in map) {
        values.push(map[key]);
    }

    return values;
}

function getKeyIdForValueFromMap(map, value) {
    var i = 0;
    for (var key in map) {
        if (key === value) {
            return i;
        } else {
            i++;
        }
    }

    return -1;
}

function idsToKey(ids) {
    var key = "";

    for (var i = 0; i < ids.length; i++) {
        if (i === ids.length - 1) {
            key += ids[i];
        } else {
            key += ids[i] + ", ";
        }
    }

    return key;
}

function hardwareListToKeys(hardwareList) {
    var key = "";

    for (var i = 0; i < hardwareList.length; i++) {
        if (i === hardwareList.length - 1) {
            key += hardwareList[i].id;
        } else {
            key += hardwareList[i].id + ", ";
        }
    }

    return key;
}
