/**
 * @license
 * Visual Blocks Editor
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
 * @fileoverview Variable input field.
 * @author fraser@google.com (Neil Fraser)
 */
'use strict';

goog.provide('Blockly.FieldVariableArray');

goog.require('Blockly.FieldDropdown');
goog.require('Blockly.Msg');
goog.require('Blockly.Variables');


/**
 * Class for a variable's dropdown field.
 * @param {?string} varname The default name for the variable.  If null,
 *     a unique variable name will be generated.
 * @param {Function} opt_changeHandler A function that is executed when a new
 *     option is selected.  Its sole argument is the new option value.  Its
 *     return value is ignored.
 * @extends {Blockly.FieldDropdown}
 * @constructor
 */

Blockly.FieldVariableArray = function(varname, opt_changeHandler, block) {

  Blockly.FieldVariableArray.superClass_.constructor.call(this,
      Blockly.FieldVariableArray.dropdownCreate, opt_changeHandler, block);

  if (varname) {
    this.setValue(varname);
  } else {
    this.setValue(Blockly.Variables.generateUniqueName());
  }
};
goog.inherits(Blockly.FieldVariableArray, Blockly.FieldDropdown);

/**
 * Clone this FieldVariableArray.
 * @return {!Blockly.FieldVariableArray} The result of calling the constructor again
 *   with the current values of the arguments used during construction.
 */
Blockly.FieldVariableArray.prototype.clone = function() {
  return new Blockly.FieldVariableArray(this.getValue(), this.changeHandler_);
};

/**
 * Get the variable's name (use a variableDB to convert into a real name).
 * Unline a regular dropdown, variables are literal and have no neutral value.
 * @return {string} Current text.
 */
Blockly.FieldVariableArray.prototype.getValue = function() {
  return this.getText();
};

/**
 * Set the variable name.
 * @param {string} text New text.
 */
Blockly.FieldVariableArray.prototype.setValue = function(text) {
  this.value_ = text;
  this.setText(text);
};

/**
 * Return a sorted list of variable names for variable dropdown menus.
 * Include a special option at the end for creating a new variable name.
 * @return {!Array.<string>} Array of variable names.
 * @this {!Blockly.FieldVariableArray}
 */
Blockly.FieldVariableArray.dropdownCreate = function(block) {

    var variableListPop = Blockly.FieldDropdown.prototype.listCreate(block, 3);

  // Ensure that the currently selected variable is an option.
  var name = this.getText();
  if (name && variableListPop.indexOf(name) == -1) {
    variableListPop.push(name);
  }
  else variableListPop.push(Blockly.Msg.SELECT_MENU);
  variableListPop.sort(goog.string.caseInsensitiveCompare);
  var options = [];
  for (var x = 0; x < variableListPop.length; x++) {
      options[x] = [variableListPop[x], variableListPop[x]];
   }
  return options;
};

/**
 * Event handler for a change in variable name.
 * Special case the 'New variable...' and 'Rename variable...' options.
 * In both of these special cases, prompt the user for a new name.
 * @param {string} text The selected dropdown menu option.
 * @return {null|undefined|string} An acceptable new variable name, or null if
 *     change is to be either aborted (cancel button) or has been already
 *     handled (rename), or undefined if an existing variable was chosen.
 * @this {!Blockly.FieldVariableArray}
 */
// Blockly.FieldVariableArray.dropdownChange = function(text) {
//   function promptName(promptText, defaultText) {
//     Blockly.hideChaff();
//     var newVar = window.prompt(promptText, defaultText);
//     // Merge runs of whitespace.  Strip leading and trailing whitespace.
//     // Beyond this, all names are legal.
//     return newVar && newVar.replace(/[\s\xa0]+/g, ' ').replace(/^ | $/g, '');
//   }
//   if (text == Blockly.Msg.RENAME_VARIABLE) {
//     var oldVar = this.getText();
//     text = promptName(Blockly.Msg.RENAME_VARIABLE_TITLE.replace('%1', oldVar),
//                       oldVar);
//     if (text) {
//       Blockly.Variables.renameVariable(oldVar, text);
//     }
//     return null;
//   } else if (text == Blockly.Msg.NEW_VARIABLE) {
//     text = promptName(Blockly.Msg.NEW_VARIABLE_TITLE, '');
//     // Since variables are case-insensitive, ensure that if the new variable
//     // matches with an existing variable, the new case prevails throughout.
//     if (text) {
//       Blockly.Variables.renameVariable(text, text);
//       return text;
//     }
//     return null;
//   }
//   return undefined;
// };


Blockly.FieldVariableArray.getBlockIdxLength = function(option) {
    var variableList = Blockly.Variables.allVariables();
    var variableListPop = []; // ????????? ????????? ?????? ???.

    for(var temp = 0 ; temp < variableList.length ; temp++) {
        if (variableList[temp][2] == option) {
            var idxLength = variableList[temp][5][0];
            return idxLength;
        }
    }
};

