import React, { Component } from 'react';
import ReactDOM from 'react-dom';
import './ConfirmModal.scss';
import classNames from 'classnames';

function renderModal(children, isOne) {
  return ReactDOM.createPortal(
    <div className={classNames('overlay', { 'one-position': isOne })}>
      {children}
    </div>,
    document.body,
  );
}

export default class ConfirmationDialog extends Component {
  render() {
    const { isOpen, onConfirm, onCancel, message, isOne } = this.props;
    if (!isOpen) {
      return null;
    }

    return renderModal(
      <div className="dialog">
        <p>{message}</p>
        <div className="buttons">
          <button onClick={onConfirm}>confirm</button>
          <button onClick={onCancel}>cancel</button>
        </div>
      </div>,
      isOne,
    );
  }
}

export class ConfirmPsdDialog extends Component {
  constructor(props) {
    super(props);
    this.state = {
      password: '',
    };
  }

  handleInputChange = (e) => {
    this.setState({ password: e.target.value });
  };

  handleConfirm = () => {
    this.props.onConfirm(this.state.password);
    this.setState({ password: '' });
  };

  handleCancel = () => {
    this.props.onCancel();
    this.setState({ password: '' });
  };

  render() {
    const { isOpen, isOne } = this.props;
    const { password } = this.state;
    if (!isOpen) {
      return null;
    }

    return renderModal(
      <div className="dialog">
        <p>Please input password:</p>
        <input
          type="password"
          value={password}
          placeholder="Please input password"
          onChange={this.handleInputChange}
        />
        <div className="buttons">
          <button onClick={this.handleConfirm}>confirm</button>
          <button onClick={this.handleCancel}>cancel</button>
        </div>
      </div>,
      isOne,
    );
  }
}

export class NotificationDialog extends Component {
  render() {
    const { isOpen, message, type } = this.props;
    if (!isOpen) {
      return null;
    }

    const notificationClass = type === 'danger'
      ? 'autoNotification danger' : 'autoNotification';

    return (
      <div className={classNames('notification-box')}>
        <div className={notificationClass}>
          <p>{message}</p>
        </div>
      </div>
    );
  }
}

export class ConfirmDoublePsdDialog extends Component {
  constructor(props) {
    super(props);
    this.state = {
      firstPwd: '',
      secondPwd: '',
    };
  }

  handleFirstPwdChange = (e) => {
    this.setState({ firstPwd: e.target.value });
  };

  handleSecondPwdChange = (e) => {
    this.setState({ secondPwd: e.target.value });
  };

  handleConfirm = () => {
    this.props.onConfirm(this.state.firstPwd, this.state.secondPwd);
    this.setState({
      firstPwd: '',
      secondPwd: '',
    });
  };

  handleCancel = () => {
    this.props.onCancel();
    this.setState({
      firstPwd: '',
      secondPwd: '',
    });
  };

  render() {
    const { isOpen, isOne } = this.props;
    const { firstPwd, secondPwd } = this.state;
    if (!isOpen) {
      return null;
    }

    return renderModal(
      <div className="dialog">
        <p>Please input password:</p>
        <input
          type="password"
          style={{ marginBottom: '10px' }}
          value={firstPwd}
          placeholder="Please input password one"
          onChange={this.handleFirstPwdChange}
        />
        <input
          type="password"
          value={secondPwd}
          placeholder="Please input password two"
          onChange={this.handleSecondPwdChange}
        />
        <div className="buttons">
          <button onClick={this.handleConfirm}>confirm</button>
          <button onClick={this.handleCancel}>cancel</button>
        </div>
      </div>,
      isOne,
    );
  }
}
