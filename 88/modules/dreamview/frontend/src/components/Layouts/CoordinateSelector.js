import React from 'react';

class CoordinateSelector extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      selectedGroup: '',
      selectedChild: null,
      currentGroup: [],
    };
  }

  componentDidUpdate(prevProps) {
    if (prevProps.options !== this.props.options) {
      this.setState(
        {
          selectedGroup: '',
          selectedChild: null,
          currentGroup: [],
        },
        () => {
          this.props.onChange(null);
        },
      );
    }
  }

  handleGroupChange = (e) => {
    this.setState(
      {
        selectedGroup: e.target.value,
        selectedChild: null,
      },
      () => {
        this.setState({
          currentGroup: this.props.options.find(
            (g) => g.value === this.state.selectedGroup,
          ),
        });
        this.props.onChange(null);
      },
    );
  };

  handleChildChange = (e) => {
    const { options } = this.props;
    const { selectedGroup } = this.state;
    const group = options.find((g) => g.value === selectedGroup);
    const childIndex = e.target.value;

    const selectedChild = group.children[childIndex].value;

    this.setState({ selectedChild }, () => {
      this.props.onChange(selectedChild);
    });
  };

  render() {
    const { options } = this.props;
    const { selectedGroup, selectedChild } = this.state;

    return (
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          marginTop: 5,
        }}
      >
        <select
          value={selectedGroup}
          onChange={this.handleGroupChange}
          style={{
            flex: 1,
            marginRight: 5,
            padding: 8,
            borderRadius: 4,
            border: '1px solid #383838',
            backgroundColor: '#000000',
            color: '#ffffff',
          }}
        >
          <option value="">Please select group!</option>
          {options.map((group) => (
            <option key={group.value} value={group.value}>
              {group.label}
            </option>
          ))}
        </select>

        <select
          value={
            selectedChild
              ? this.state.currentGroup.children.findIndex(
                (c) => c.value === selectedChild,
              )
              : ''
          }
          onChange={this.handleChildChange}
          disabled={!selectedGroup}
          style={{
            flex: 1,
            padding: 8,
            borderRadius: 4,
            border: '1px solid #383838',
            backgroundColor: '#000000',
            color: '#ffffff',
            cursor: selectedGroup ? 'pointer' : 'not-allowed',
          }}
        >
          <option value="">Please select point!</option>
          {this.state.currentGroup &&
            this.state.currentGroup.children &&
            this.state.currentGroup.children.map((child, index) => (
              <option key={index} value={index}>
                {child.label}
              </option>
            ))}
        </select>
      </div>
    );
  }
}

export default CoordinateSelector;
