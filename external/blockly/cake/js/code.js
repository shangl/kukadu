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
jQuery(function ($, undefined) {
    $('#terminal').terminal(function (command, term) {
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
        prompt: 'cake> '
    });
});

/**
 * Save current codes into a *.c file.
 * https://github.com/eligrey/FileSaver.js
 */
function downloadCode() {
    var code = Blockly.cake.workspaceToCode();
    var codeArray = [];
    codeArray.push(code);
    var codeBlob = new Blob(codeArray, {type: "text/plain;charset=utf-8"});
    saveAs(codeBlob, "code.c");
}

function downloadBlocks() {
    var xml = Blockly.Xml.workspaceToDom(Blockly.mainWorkspace);
    return Blockly.Xml.domToText(xml);
}

function reloadBlocks(xmlText) {

    loading = true;

    Blockly.mainWorkspace.clear();
    var dom = Blockly.Xml.textToDom(xmlText);
    var domCopy = Blockly.Xml.textToDom(xmlText);

    var allBlocks = dom.getElementsByTagName("block");
    var allBlocksCopy = domCopy.getElementsByTagName("block");

    var skillLoaderBlocks = [];

    var j = 0;
    for (var i = 0; i < allBlocks.length; i++) {
        if (allBlocks[i].getAttribute("type") === "skillloader") {
            var childBlocks = allBlocks[i].childNodes;
            for (var k = 0; k < childBlocks.length; k++) {
                var blockName = childBlocks[k].getAttribute("name");
                if (blockName !== null && (blockName === "SkillOptions" || blockName.substr(0, 9) === "attribute")) {
                    childBlocks[k].parentNode.removeChild(childBlocks[k]);
                    k = k - 1;
                }
            }

            skillLoaderBlocks[j++] = allBlocksCopy[i];
        }
    }

    Blockly.Xml.domToWorkspace(Blockly.mainWorkspace, dom);
    loading = false;

    var allBlocksInWorkspace = Blockly.mainWorkspace.getAllBlocks();
    var blocksToModify = [];

    for (var i = 0; i < allBlocksInWorkspace.length; i++) {
        if (allBlocksInWorkspace[i].type === "skillloader") {
            blocksToModify.push(allBlocksInWorkspace[i]);
        } else {
            if(allBlocksInWorkspace[i].onchange) {
                allBlocksInWorkspace[i].onchange();
            }
        }
    }

    for (var i = 0; i < skillLoaderBlocks.length; i++) {
        var block = blocksToModify[i];
        var childBlocks = skillLoaderBlocks[i].childNodes;

        for (var k = 0; k < childBlocks.length; k++) {
            var name = childBlocks[k].getAttribute('name');
            if (name != null && (name === "SkillOptions" || name.substr(0, 9) === "attribute")) {
                var content = childBlocks[k].textContent;

                block.onchange();
                block.setFieldValue(content, name);
            }
        }
    }
}

function getCurrentSkillName() {
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

function initializeDatabaseloader(jsonString) {
    Databaseloader.init(JSON.parse(jsonString));
}