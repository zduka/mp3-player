
/* Shows messages at the top of the page. 
   
   Nothing fancy, the messages can be dismissed by the user, or will be deleted automatically after 5 seconds. 
 */
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

// Status update

/** Reads the status from the player and displays it 
 */
function getStatus() {
    $('#panel-status').show();
    $('#panel-status-updating').text("updating");
    $('#panel-status-updating').show();
    // vcc temp mem charging batt headphones maxLoopTime ssid rssi ap
    /*
    $.getJSON("status", function(data) {
        $('#panel-status-updating').hide();

    });
    */
}

/** Reads the settinsg from the player and displays them.
 */
function getSettings() {

}







//showMessage("script loaded");


/** Loads and optionally converts the mp3 file to be added to a playlist.
 */
function mp3Upload() {
    console.log(this.files);
    f = this.files[0]
    console.log(f.size);
    let reader = new FileReader();
    reader.onload = function() {
        let buffer = reader.result;
        console.log("Loaded " + buffer.byteLength);
    };
    reader.readAsArrayBuffer(f);    
}

/** Called when the whole page has been loaded from ESP. Note that this may take some time.
 */
$( document ).ready(function() {
    $('#alertsSpace').empty(); // clear the please wait message
    getStatus();
        //showMessage("Ready");
    //$('#mp3-uploader').change(mp3Upload)
    //console.log( "ready!" );
});

