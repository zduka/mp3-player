

let messageIndex = 0;

function showMessage(message, kind = "info") {
    let aname = "message" + messageIndex;
    messageIndex = messageIndex + 1;
    $('#alertsSpace').append(
        '<div id="' + aname + '" class="alert alert-' + kind + '"><a class="close" data-dismiss="alert">×</a><span>'+message+'</span></div>' 
    )
    setTimeout(function() {
        $('#' + aname).remove()
    }, 5000);
}

function showError(message) {
    showMessage(message, "danger");
}

function showWarning(message) {
    showMessage(message, "wanring")
}

function showSuccess(message) {
    showMessage(message, "success");
}








//showMessage("script loaded");