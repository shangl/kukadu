'use strict';
// Depending on the URL argument, render as LTR or RTL.
var rtl = (document.location.search == '?rtl');
var block = null;

function start() {
  Blockly.inject(document.getElementById('blocklyDiv'), 
      {
        path: '../', 
        toolbox: document.getElementById('toolbox')
      }
  );
  Blockly.addChangeListener(renderContent);
}

function renderContent() {
  var content = document.getElementById('code');
  var code = Blockly.cake.workspaceToCode();
  content.textContent = code;
  if (typeof prettyPrintOne == 'function') {
    code = content.innerHTML;
    code = prettyPrintOne(code, 'c');
    content.innerHTML = code;
  }
}

/**
 * Discard all blocks from the workspace.
 */
function discard() {
  var count = Blockly.mainWorkspace.getAllBlocks().length;
  if (count < 2 || window.confirm("Remove all blocks?")) {
    Blockly.mainWorkspace.clear();
    window.location.hash = '';
  }
}

/**
 * Insert terminal into page.
 * https://github.com/jcubic/jquery.terminal
 */
jQuery(function($, undefined) {
    $('#terminal').terminal(function(command, term) {
        if (command !== '') {
            var result = window.eval(command);
            if (result != undefined) {
                term.echo(String(result));
            }
        }
    }, {
        greetings: 'Cake Console Terminal',
        name: 'js_demo',
        height: 0,
        width: 0,
        prompt: 'cake> '});
});

/**
 * Save current codes into a *.c file.
 * https://github.com/eligrey/FileSaver.js
 */
function downloadCode() {
  var code = Blockly.cake.workspaceToCode();
  var codeArray = [];
  codeArray.push(code);
  console.log(code);
  var codeBlob = new Blob(codeArray, {type: "text/plain;charset=utf-8"});
  saveAs(codeBlob, "code.c");
}

function downloadBlocks() {
    var xml = Blockly.Xml.workspaceToDom(Blockly.mainWorkspace);
    return Blockly.Xml.domToText(xml);
    /*
    var all = [];
    all["html"] = document.getElementById("main-wrapper").innerHTML;
    all["blockly"] = Blockly;
    //var desAll =  JSON.parse(ws);
    Blockly = all["blockly"];
    document.getElementById("main-wrapper").innerHTML = all["html"];
    console.log(Blockly.mainWorkspace.getAllBlocks());
    $(Blockly.mainWorkspace.getAllBlocks()).each(function() { console.log(this); this.init(); });
*/

}

function reloadBlocks(xmlText) {

    console.log(Blockly.mainWorkspace);
    loading = true;

    //Blockly.Blocks['skillloader'].onchange = doNothing;
    var dom = Blockly.Xml.textToDom(xmlText);
    Blockly.Xml.domToWorkspace(Blockly.mainWorkspace, dom);
    //Blockly.Blocks['skillloader'].onchange = realOnChange;

    loading = false;

}

function getCurrentSkillName(){
    return Blockly.cake.activeSkill_;
}

function isSkillInstalled() {
    return Blockly.cake.installSkill === 'TRUE';
}



//Do not delete these unused functions - they are used from the c++ application
function getExecutionMode() {
    return Blockly.cake.executionMode;
}

function getCode() {
    return Blockly.cake.workspaceToCode();
}

function initializeDatabaseloader(jsonString){
  Databaseloader.init(JSON.parse(jsonString));
}