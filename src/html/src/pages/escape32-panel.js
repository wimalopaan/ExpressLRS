import {html, LitElement} from "lit"
import {customElement, state} from "lit/decorators.js"
import '../components/filedrag.js'
import {post, showAlert, showConfirm} from "../utils/feedback.js"
import {elrsState, formatBand, formatWifiRssi} from "../utils/state.js"

@customElement('escape32-panel')
class UpdatePanel extends LitElement {
    @state() accessor progress = 0
    @state() accessor progressText = ''

    createRenderRoot() {
        this._completeHandler = this._completeHandler.bind(this)
        this._progressHandler = this._progressHandler.bind(this)
        this._errorHandler = this._errorHandler.bind(this)
        this._abortHandler = this._abortHandler.bind(this)
        return this
    }

    render() {
        return html`
            <div class="mui-panel mui--text-title">Information</div>
            <div class="mui-panel">
                <table class="mui-table mui-table--bordered">
                    <tbody>
                    <tr><td><b>Firmware</b></td><td>${elrsState.settings.escape32_fw}</td></tr>
                    <tr><td><b>Bootloader</b></td><td>${elrsState.settings.escape32_bl}</td></tr>
                    <tr><td><b>Target</b></td><td>${elrsState.settings.escape32_target}</td></tr>
                    <tr><td><b>Status</b></td><td>${elrsState.settings.escape32_status}</td></tr>
                    <tr><td><b>Update</b></td><td>${elrsState.settings.escape32_update}</td></tr>
                    </tbody>
                </table>
            </div>
            <div class="mui-panel mui--text-title">ESCape32 Firmware (Bootloader) Update</div>
            <div class="mui-panel">
                <file-drop id="firmware-upload" label="Select firmware file" @file-drop="${this._fileSelectHandler}">or drop firmware file here</file-drop>
                <br/>
                <h3 id="status">${this.progressText}</h3>
                <progress id="progressBar" value="0" max="100" style="width:100%;" .value="${this.progress}"></progress>
            </div>
        `
    }
           
    _fileSelectHandler(e) {
        const files = e.detail.files
        const fileExt = files[0].name.split('.').pop()
        const expectedFileExt = 'bin'
        const expectedFileExtDesc = '.bin file.'
        if (fileExt === expectedFileExt) {
            this._uploadFile(files[0])
        } else {
            showAlert('error', 'Incorrect File Format', 'You selected the file &quot;' + files[0].name.toString() + '&quot;.<br />The firmware file must be a ' + expectedFileExtDesc)
        }
    }

    _uploadFile(file) {
        post('/ESCape32Upload', file, {
            onprogress: this._progressHandler,
            onload: this._completeHandler,
            onerror: this._errorHandler,
            onabort: this._abortHandler,
        })
    }

    _progressHandler(event) {
        const percent = Math.round((event.loaded / event.total) * 100)
        this.progress = percent
        this.progressText = percent + '% uploaded... please wait'
    }

    _completeHandler(request) {
        this._resetProgress()
        const data = JSON.parse(request.responseText)
        if (data.status === 'ok') {
            this._showFlashingProgress(data.msg, 1)
        }
        else if (data.status === 'ok_boot') {        
            this._showFlashingProgress(data.msg, 5)
        } else {
            this._showAlert('error', 'Update Failed', data.msg)
        }
    }

    _errorHandler(request) {
        return this._showAlert('error', 'Update Failed', request.responseText)
    }

    _abortHandler(request) {
        return this._showAlert('info', 'Update Aborted', request.responseText)
    }

    _resetProgress() {
        this.progressText = ''
        this.progress = 0
    }

    _showAlert(type, title, message) {
        this._resetProgress()
        return showAlert(type, title, message)
    }

    _showFlashingProgress(message, increment) {
        let percent = 0
        const interval = setInterval(() => {
            percent = percent + increment
            this.progress = percent
            this.progressText = percent + '% flashed... please wait'
            if (percent === 100) {
                clearInterval(interval)
                this._showAlert('success', 'Update Succeeded', message)
            }
        }, 100)
    }
}
