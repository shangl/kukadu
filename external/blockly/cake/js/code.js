'use strict';
// Depending on the URL argument, render as LTR or RTL.
var rtl = (document.location.search == '?rtl');
var block = null;

var setFieldValueError = false;

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
	setFieldValueError = false;

	Blockly.mainWorkspace.clear();
	
	$('#xmlparsediv').html(xmlText);
	$('#xmlparsediv field[name*="attribute"]').remove();
	
	var dom = Blockly.Xml.textToDom($('#xmlparsediv').html());
	Blockly.Xml.domToWorkspace(Blockly.mainWorkspace, dom, true);
	
	loading = false;

	var currentId = 0;
	$('#xmlparsediv').html(xmlText);
	$(Blockly.mainWorkspace.getAllBlocks()).each(function(idx, el) {
		
		if(el.type == "skillloader") {
			
			var dataElement = $('#xmlparsediv block[id="' + el.id + '"]');
			var selectedSkill = dataElement.find('[name=SkillOptions]').html();
			el.onchange();
			el.setFieldValue(selectedSkill, "SkillOptions");
			el.onchange();
			
			$('#xmlparsediv block[id="' + el.id + '"]').children('field[name*="attribute"]').each(
				function(idx, attributeElement) {
					var jqEl = $(attributeElement);
					var attributeName = jqEl.attr("name");
					var attributeValue = jqEl.html();
					el.onchange();
					el.setFieldValue(attributeValue, attributeName);
					el.onchange();
				}
			);
			
		}
		
	});
	
	if(setFieldValueError)
		alert("warning: there were errors loading the file (check the debug console");

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
